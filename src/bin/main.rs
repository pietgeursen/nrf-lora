#![no_std]
#![no_main]

use nrf52840_hal::gpio::{Disconnected, Level, Output, Pin, PushPull};
use nrf52840_hal::pac::{DWT, SPIM0, TIMER3, TWIM0};
use nrf52840_hal::prelude::*;
use nrf52840_hal::spim::{
    Frequency as SpimFrequency, Mode as SpimMode, Phase, Pins as SpimPins, Polarity,
};
use nrf52840_hal::timer::OneShot;
use nrf52840_hal::Timer;
use nrf52840_hal::{gpio::p0::Parts as Parts0, gpio::p1::Parts as Parts1, gpiote::*};
use nrf52840_hal::{Spim, Twim};
use nrf_bamboo_rs as _;
use rfm95_rs::{
    lora::{
        dio_mapping::*,
        fei::*,
        frequency_rf::*,
        irq_flags::IrqFlags,
        irq_masks::{IrqMask, IrqMasks},
        modem_config1::{Bandwidth, CodingRate, ModemConfig1},
        modem_config2::SpreadingFactor,
        modem_config3::{LowDataRateOptimize, ModemConfig3},
        op_mode::Mode,
        pa_config::{PaConfig, PaSelect},
    },
    size_bytes::SizeBytes,
    Config as RfmConfig, LoRaMode, RFM95,
};
use rtic::app;

//use apds9960::*;
//use bamboo_rs_core::entry::*;
//use bamboo_rs_core::*;
use bmp280_rs::*;

use rtic::cyccnt::U32Ext as _;
const PERIOD: u32 = 128_000_000;

const TX_FREQUENCY_HZ: u32 = 915_000_000;

pub enum RfmActions {
    TxCompleted,
    RxCompleted,
    TxRetryButtonPressed,
    StartTxCommandReceived,
    StartRxCommandReceived,
    StartCadCommandReceived,
    CadDetected,
    CadTimeout,
}

pub enum RfmStateMachine {
    Idle,
    Transmitting,
    TransmitComplete,
    Receiving,
    ReceivingComplete,
    DetectingCad,
    CadDetected,
}

impl RfmStateMachine {
    pub fn dispatch<OnExit: FnMut(&Self), OnEnter: FnMut(&Self)>(
        &mut self,
        action: &RfmActions,
        mut onExit: OnExit,
        mut onEnter: OnEnter,
    ) {
        match self {
            RfmStateMachine::Idle => match action {
                RfmActions::StartTxCommandReceived => {
                    onExit(self);
                    *self = RfmStateMachine::Transmitting;
                    onEnter(self);
                }
                _ => (),
            },
            RfmStateMachine::Transmitting => {}
            RfmStateMachine::TransmitComplete => {}
            RfmStateMachine::Receiving => {}
            RfmStateMachine::ReceivingComplete => {}
            RfmStateMachine::DetectingCad => {}
            RfmStateMachine::CadDetected => {}
        }
    }
}

#[app(device = nrf52840_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        rfm_state_machine: RfmStateMachine,
        status_led: Pin<Output<PushPull>>,
        gpiote: Gpiote,
        spim: Spim<SPIM0>,
        rfm95: RFM95<
            Spim<SPIM0>,
            LoRaMode,
            Pin<Output<PushPull>>,
            Pin<Output<PushPull>>,
            Timer<TIMER3, OneShot>,
        >,
        ppm_correction: Option<PpmCorrection>,
        rf_correction: Option<FrequencyRf>,
    }

    #[idle(resources=[spim, rfm95])]
    fn idle(ctx: idle::Context) -> ! {
        loop {
            //defmt::info!("idle");
            //cortex_m::asm::wfi();
        }
    }

    #[init(schedule = [measure_bmp_pressure], spawn=[tx_data])]
    fn init(ctx: init::Context) -> init::LateResources {
        // Cortex-M peripherals
        defmt::info!("init");
        let mut core = ctx.core;

        // Device specific peripherals
        let device: nrf52840_hal::pac::Peripherals = ctx.device;
        // Initialize (enable) the monotonic timer (CYCCNT)
        defmt::trace!("configure sys timer");
        configure_systimer(&mut core);

        let p0 = Parts0::new(device.P0);
        let p1 = Parts1::new(device.P1);

        let status_led = p1.p1_10.into_push_pull_output(Level::Low).degrade();

        let gpiote = Gpiote::new(device.GPIOTE);

        let switch = p1.p1_02.into_pullup_input().degrade();
        gpiote
            .channel1()
            .input_pin(&switch)
            .lo_to_hi()
            .enable_interrupt();

        let mut spim = create_spim(
            device.SPIM0,
            p0.p0_13.degrade(),
            p0.p0_12.degrade(),
            p0.p0_15.degrade(),
        );

        let delay_timer = Timer::new(device.TIMER3);

        // Feather pin D12
        let spi_cs = p0.p0_08.into_push_pull_output(Level::High).degrade();
        // Feather pin D11
        let rfm_irq = p0.p0_06.into_pullup_input().degrade();
        // Feather pin D10
        let rfm_reset = p0.p0_27.into_push_pull_output(Level::High).degrade();

        let config = RfmConfig {
            implicit_header_mode_on: false,
            bandwidth: Bandwidth::K125,
            coding_rate: CodingRate::Rate4_8,
            low_frequency_mode: false,
            rx_payload_crc_on: true,
            spreading_factor: SpreadingFactor::Twelve,
        };

        // Enable interrupt from rfm d0 interrrupt pin.
        gpiote
            .channel0()
            .input_pin(&rfm_irq)
            .lo_to_hi()
            .enable_interrupt();

        defmt::trace!("enabled rfm irq interrupt");

        let mut rfm95 = RFM95::new(&mut spim, spi_cs, rfm_reset, config, delay_timer).unwrap();
        initialize_rfm95(&mut rfm95, &mut spim);
        defmt::trace!("created rfm95");

        ctx.spawn.tx_data().unwrap();

        init::LateResources {
            ppm_correction: None,
            rf_correction: None,
            rfm_state_machine: RfmStateMachine::Idle,
            status_led,
            gpiote,
            rfm95,
            spim,
        }
    }

    #[task(resources=[rfm95, spim, rfm_state_machine], capacity=4)]
    fn tx_data(mut ctx: tx_data::Context) {
        defmt::trace!("tx_data");
        // clear the tx interrupt in the rfm95
        let mut data_to_transmit = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];

        ctx.resources
            .rfm95
            .read_update_write_packed_struct::<_, _, { DioMapping::SIZE }>(
                &mut ctx.resources.spim,
                |mapping: &mut DioMapping| {
                    mapping.dio_0_mapping = Dio0Mapping::TxDone;
                },
            )
            .unwrap();

        // Set the tx freq back to 915.
        ctx.resources
            .rfm95
            .set_frequency(
                &mut ctx.resources.spim,
                Frequency::new::<hertz>(TX_FREQUENCY_HZ),
            )
            .unwrap();

        *ctx.resources.rfm_state_machine = RfmStateMachine::Transmitting;

        ctx.resources
            .rfm95
            .transmit_data(&mut ctx.resources.spim, &mut data_to_transmit)
            .unwrap();
    }

    #[task(resources=[rfm95, spim, rfm_state_machine, ppm_correction, rf_correction])]
    fn rx_data(mut ctx: rx_data::Context) {
        defmt::trace!("rx_data");
        // clear the tx interrupt in the rfm95
        ctx.resources
            .rfm95
            .read_update_write_packed_struct::<_, _, { DioMapping::SIZE }>(
                &mut ctx.resources.spim,
                |mapping: &mut DioMapping| {
                    mapping.dio_0_mapping = Dio0Mapping::RxDone;
                },
            )
            .unwrap();

        if let Some(rf_correction) = ctx.resources.rf_correction {
            defmt::trace!("updating rf_correction");
            //let default_freq = Frequency::new::<hertz>(TX_FREQUENCY_HZ);
            ctx.resources
                .rfm95
                .write_packed_struct::<FrequencyRf>(&mut ctx.resources.spim, rf_correction)
                .unwrap();
        }

        if let Some(ppm_correction) = ctx.resources.ppm_correction {
            defmt::trace!("updating ppm_correction");
            ctx.resources
                .rfm95
                .write_packed_struct::<_>(&mut ctx.resources.spim, ppm_correction)
                .unwrap();
        }

        ctx.resources
            .rfm95
            .receive_data(&mut ctx.resources.spim)
            .unwrap();

        *ctx.resources.rfm_state_machine = RfmStateMachine::Receiving;
    }

    #[task(resources=[rfm95, spim, rfm_state_machine], spawn=[rx_data])]
    fn tx_complete(mut ctx: tx_complete::Context) {
        defmt::trace!("tx_complete");
        ctx.resources
            .rfm95
            .read_update_write_packed_struct::<_, _, { IrqFlags::SIZE }>(
                &mut ctx.resources.spim,
                |masks: &mut IrqFlags| {
                    masks.tx_done = true;
                },
            )
            .unwrap();

        *ctx.resources.rfm_state_machine = RfmStateMachine::Idle;
        ctx.spawn.rx_data().unwrap();
    }

    #[task(schedule=[tx_data], resources=[rfm95, spim, rfm_state_machine, status_led, rf_correction, ppm_correction], spawn=[tx_data])]
    fn rx_complete(mut ctx: rx_complete::Context) {
        defmt::trace!("rx_complete");
        // clear the rx interrupt in the rfm95
        let irq_flags = ctx
            .resources
            .rfm95
            .read_packed_struct::<IrqFlags, { IrqFlags::SIZE }>(&mut ctx.resources.spim)
            .unwrap();

        ctx.resources
            .rfm95
            .read_update_write_packed_struct::<_, _, { IrqFlags::SIZE }>(
                &mut ctx.resources.spim,
                |masks: &mut IrqFlags| {
                    masks.rx_done = true;
                    masks.payload_crc_error = true;
                    masks.rx_timeout = true;
                },
            )
            .unwrap();

        let frq_err = ctx
            .resources
            .rfm95
            .read_packed_struct::<Fei, { Fei::SIZE }>(&mut ctx.resources.spim)
            .unwrap();

        let bandwidth = ctx
            .resources
            .rfm95
            .read_packed_struct::<ModemConfig1, { ModemConfig1::SIZE }>(&mut ctx.resources.spim)
            .unwrap()
            .bandwidth;

        let current_freq = ctx
            .resources
            .rfm95
            .read_packed_struct::<FrequencyRf, { FrequencyRf::SIZE }>(&mut ctx.resources.spim)
            .unwrap();

        let rf_correction = frq_err.frf_new(&current_freq, &bandwidth);
        let ppm_correction = frq_err.ppm_correction(&current_freq, &bandwidth);
        *ctx.resources.rf_correction = Some(rf_correction);
        *ctx.resources.ppm_correction = Some(ppm_correction);

        if irq_flags.rx_done && !irq_flags.payload_crc_error {
            defmt::info!("valid rx complete");
            toggle_status_led(ctx.resources.status_led);

            ctx.resources
                .rfm95
                .set_mode(ctx.resources.spim, Mode::Standby)
                .unwrap();

            *ctx.resources.rfm_state_machine = RfmStateMachine::Idle;
            ctx.schedule
                .tx_data(ctx.scheduled + PERIOD.cycles())
                .unwrap();
        }
    }

    #[task(resources=[rfm_state_machine], spawn=[tx_complete, rx_complete])]
    fn handle_rfm_interrupt(ctx: handle_rfm_interrupt::Context) {
        defmt::trace!("handle_rfm_interrupt");
        match ctx.resources.rfm_state_machine {
            RfmStateMachine::Transmitting => {
                *ctx.resources.rfm_state_machine = RfmStateMachine::TransmitComplete;
                ctx.spawn.tx_complete().unwrap();
            }
            RfmStateMachine::Receiving => {
                *ctx.resources.rfm_state_machine = RfmStateMachine::ReceivingComplete;
                ctx.spawn.rx_complete().unwrap();
            }
            _ => {
                defmt::info!("in handle_rfm_interrupt with unexpected state")
            }
        }
    }

    #[task(resources=[rfm_state_machine], spawn=[tx_data])]
    fn handle_retry_interrupt(ctx: handle_retry_interrupt::Context) {
        defmt::trace!("handle_rfm_interrupt");
        ctx.spawn.tx_data().unwrap();
    }

    #[task(binds = GPIOTE, resources = [gpiote], priority=2, spawn=[handle_rfm_interrupt, handle_retry_interrupt])]
    fn on_gpiote(ctx: on_gpiote::Context) {
        if ctx.resources.gpiote.channel0().is_event_triggered() {
            defmt::trace!("Interrupt from channel 0 event");
            ctx.spawn.handle_rfm_interrupt().unwrap();
        }
        if ctx.resources.gpiote.channel1().is_event_triggered() {
            defmt::trace!("Interrupt from channel 1 event");
            ctx.spawn.handle_retry_interrupt().unwrap();
        }

        if ctx.resources.gpiote.port().is_event_triggered() {
            defmt::trace!("Interrupt from port event");
        }
        // Reset all events
        ctx.resources.gpiote.reset_events();
    }

    #[task(schedule = [measure_bmp_pressure], resources = [status_led])]
    fn measure_bmp_pressure(ctx: measure_bmp_pressure::Context) {
        defmt::info!("measure_bmp_pressure");
        //        let mut twim = ctx.resources.twim;
        //        twim.enable();
        //
        //        let pressure = ctx.resources.bmp280.read_pressure(&mut twim);
        //        defmt::info!("pressure: {:?}", pressure);
        //        if let Ok(p) = pressure {
        //            defmt::info!("pressure: {:?} pa", p / 256);
        //        }
        //
        //        let temperature = ctx.resources.bmp280.read_temperature(&mut twim);
        //        twim.disable();
        //        defmt::info!("temperature: {:?}", temperature);

        toggle_status_led(ctx.resources.status_led);

        ctx.schedule
            .measure_bmp_pressure(ctx.scheduled + PERIOD.cycles())
            .unwrap();
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    extern "C" {
        fn I2S();
    }
};

fn initialize_rfm95(
    rfm95: &mut RFM95<
        Spim<SPIM0>,
        LoRaMode,
        Pin<Output<PushPull>>,
        Pin<Output<PushPull>>,
        Timer<TIMER3>,
    >,
    spim: &mut Spim<SPIM0>,
) {
    rfm95.init(spim).unwrap();
    rfm95
        .set_frequency(spim, Frequency::new::<hertz>(TX_FREQUENCY_HZ))
        .unwrap();
    defmt::trace!("set rfm95 freq");
    rfm95
        .read_update_write_packed_struct::<_, _, { IrqMasks::SIZE }>(
            spim,
            |masks: &mut IrqMasks| {
                masks.tx_done = IrqMask::Enabled;
                masks.rx_done = IrqMask::Enabled;
            },
        )
        .unwrap();
    rfm95
        .read_update_write_packed_struct::<_, _, { PaConfig::SIZE }>(
            spim,
            |config: &mut PaConfig| {
                config.pa_select = PaSelect::PaBoost;
            },
        )
        .unwrap();
    rfm95
        .read_update_write_packed_struct::<_, _, { ModemConfig3::SIZE }>(
            spim,
            |config: &mut ModemConfig3| {
                //config.agc = AGC::On;
                config.low_data_rate_optimize = LowDataRateOptimize::On;
            },
        )
        .unwrap();
}

fn create_spim(
    spim0: SPIM0,
    mosi_pin: Pin<Disconnected>,
    miso_pin: Pin<Disconnected>,
    sck_pin: Pin<Disconnected>,
) -> Spim<SPIM0> {
    let mosi_pin = mosi_pin.into_push_pull_output(Level::High);
    let miso_pin = miso_pin.into_floating_input();
    let sck_pin = sck_pin.into_push_pull_output(Level::High);
    let spim_pins = SpimPins {
        sck: sck_pin,
        mosi: Some(mosi_pin),
        miso: Some(miso_pin),
    };
    let mode = SpimMode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    Spim::new(spim0, spim_pins, SpimFrequency::K500, mode, 0xFF)
}

fn toggle_status_led(led: &mut Pin<Output<PushPull>>) {
    if led.is_set_high().unwrap() {
        led.set_low().unwrap();
    } else {
        led.set_high().unwrap();
    }
}

fn configure_systimer(core: &mut rtic::Peripherals) {
    core.DCB.enable_trace();
    DWT::unlock();
    core.DWT.enable_cycle_counter();
}

//fn create_twim0(p0: &mut Parts, twim0: TWIM0) -> Twim<TWIM0> {
//    let scl_pin = p0.p0_11.into_floating_input();
//    let sda_pin = p0.p0_12.into_floating_input();
//    let twim_pins = TwimPins {
//        scl: scl_pin.degrade(),
//        sda: sda_pin.degrade(),
//    };
//    Twim::new(twim0, twim_pins, FREQUENCY_A::K100)
//}
//
fn create_bmp280(twim: &mut Twim<TWIM0>) -> BMP280<Twim<TWIM0>, ModeNormal> {
    BMP280::new(
        twim,
        I2CAddress::SdoPulledUp,
        Config::handheld_device_lowpower(),
    )
    .unwrap()
    .into_normal_mode(twim)
    .unwrap()
}
//#[entry]
//fn main() -> ! {
//    let periph = Peripherals::take().unwrap();
//    periph.P1.dir.write(|w| w.pin10().output());
//
////    let p0 = Parts::new(periph.P0);
////    let scl_pin = p0.p0_11.into_floating_input();
////    let sda_pin = p0.p0_12.into_floating_input();
////    let twim_pins = TwimPins{ scl: scl_pin.degrade(), sda: sda_pin.degrade() };
////    let twim = Twim::new(periph.TWIM0, twim_pins, FREQUENCY_A::K100);
////
////    let bmp_config = Config::handheld_device_lowpower();
////    let mut bmp280 = BMP280::new(twim, I2CAddress::SdoPulledUp, bmp_config )
////        .unwrap()
////        .into_normal_mode()
////        .unwrap();
//
//    let p0 = Parts::new(periph.P0);
//    let scl_pin = p0.p0_11.into_floating_input();
//    let sda_pin = p0.p0_12.into_floating_input();
//    let twim_pins = TwimPins{ scl: scl_pin.degrade(), sda: sda_pin.degrade() };
//    let twim = Twim::new(periph.TWIM0, twim_pins, FREQUENCY_A::K100);
//
//    let mut apds9960 = Apds9960::new(twim);
//    apds9960.enable().unwrap();
//    apds9960.enable_light().unwrap();
//
//    let mut is_enabled = 0;
//    let err_code = unsafe{ sd_softdevice_is_enabled(&mut is_enabled)};
//    defmt::info!("is_enabled: {:?}", is_enabled);
//    defmt::info!("err code: {:?}", err_code);
//
//    defmt::info!("hellllloooo");
//    let mut out = [0; MAX_ENTRY_SIZE];
//    let mut out2 = [0; MAX_ENTRY_SIZE];
//    let payload = [1, 2, 3, 4, 5];
//
//    let secret_key_bytes = [
//        197, 236, 75, 1, 28, 156, 231, 168, 29, 26, 12, 113, 0, 150, 235, 94, 140, 223, 220, 213,
//        102, 242, 213, 42, 128, 46, 137, 204, 44, 53, 206, 8,
//    ];
//
//    let public_key_bytes = [
//        221, 153, 125, 189, 92, 63, 192, 146, 29, 154, 178, 208, 108, 47, 58, 74, 149, 140, 115,
//        129, 117, 166, 223, 169, 171, 72, 94, 32, 190, 154, 67, 189,
//    ];
//
//    let public = PublicKey::from_bytes(&public_key_bytes).unwrap();
//    let secret = SecretKey::from_bytes(&secret_key_bytes).unwrap();
//    let key_pair = Keypair { public, secret };
//
//    let result1 = publish(
//        &mut out,
//        &key_pair,
//        1,
//        &payload,
//        false,
//        None,
//        None,
//        None,
//    );
//    let entry1 = decode(&out).unwrap();
//    defmt::info!("published first entry result was ok: {:?}", result1.is_ok());
//    defmt::info!("published first entry was: {:?}", entry1.seq_num as u8);
//
//    let result2 = publish(
//        &mut out2,
//        &key_pair,
//        1,
//        &payload,
//        false,
//        Some(entry1.seq_num),
//        Some(&out),
//        Some(&out),
//    );
//    let entry2 = decode(&out2).unwrap();
//
//    defmt::info!("published second entry result was: {:?}", result2.is_ok());
//    defmt::info!("published first entry was: {:?}", entry2.seq_num as u8);
//
//    loop {
//        let result = verify(&out2, Some(&payload), Some(&out), Some(&out)).is_ok();
//        assert!(result);
//        periph.P1.out.modify(|r, w| {
//            let current = r.pin10().bit();
//            w.pin10().bit(!current)
//        });
//
//        apds9960.read_light().unwrap();
//        //let temp = bmp280.read_temperature().unwrap();
//        //defmt::info!("temp was: {:?}", temp);
//
//        //let pressure = bmp280.read_pressure().unwrap();
//        //defmt::info!("pressure was: {:?}", pressure);
//    }
//}
