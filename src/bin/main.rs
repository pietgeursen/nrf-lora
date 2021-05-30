#![no_std]
#![no_main]

use core::mem;
use nrf52840_hal::{
    gpio::{
        p0::Parts as Parts0, p1::Parts as Parts1, Disconnected, Input, Level, Output, Pin, PullUp,
        PushPull,
    },
    gpiote::*,
    pac::{RTC1, SPIM0},
    prelude::*,
    rtc::{Rtc, RtcCompareReg, RtcInterrupt},
    spim::{Frequency as SpimFrequency, Mode as SpimMode, Phase, Pins as SpimPins, Polarity},
    Spim, Timer,
};

use nrf_bamboo_rs as _;
use nrf_bamboo_rs::ble::*;
use nrf_bamboo_rs::ble::Config as BleConfig;
use nrf_bamboo_rs::rfm_statemachine::*;

use rfm95_rs::{
    lora::{
        modem_config1::{Bandwidth, CodingRate},
        modem_config2::SpreadingFactor,
    },
    Config as RfmConfig, RFM95,
};
use rtic::app;

#[app(device = nrf52840_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        status_led: Pin<Output<PushPull>>,
        gpiote: Gpiote,
        spim: Spim<SPIM0>,
        statemachine: StateMachine<Context>,
        rtc1: Rtc<RTC1>,
        rfm_irq_pin: Pin<Input<PullUp>>,
        switch_pin: Pin<Input<PullUp>>,
    }

    #[idle(resources=[])]
    fn idle(_ctx: idle::Context) -> ! {
        let ble_config = BleConfig{
            gap_device_name: Some(ble_gap_cfg_device_name_t {
                p_value: b"HelloRust" as *const u8 as _,
                current_len: 9,
                max_len: 9,
                write_perm: unsafe { mem::zeroed() },
                _bitfield_1: ble_gap_cfg_device_name_t::new_bitfield_1(
                    BLE_GATTS_VLOC_STACK as u8,
                ),
            }),
            ..Default::default()
        };
        configure_ble(&ble_config);

        let mut adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET as u8;
        
        #[rustfmt::skip]
        let mut adv_data = [
            0x02, 0x01, BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
            0x03, 0x03, 0x09, 0x18,
            0x0a, 0x09, b'H', b'e', b'l', b'l', b'o', b'R', b'u', b's', b't',
        ];
        #[rustfmt::skip]
        let mut scan_data = [
            0x03, 0x03, 0x09, 0x18,
        ];

        let gap_adv_data = ble_gap_adv_data_t{
            adv_data: ble_data_t{p_data: adv_data.as_mut_ptr() , len: adv_data.len() as u16},
            scan_rsp_data: ble_data_t{p_data: scan_data.as_mut_ptr(), len: scan_data.len() as u16} 
        };

        let mut gap_adv_params: ble_gap_adv_params_t = unsafe{mem::zeroed()};

        gap_adv_params.properties.type_ = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED as u8;
        gap_adv_params.primary_phy = BLE_GAP_PHY_AUTO as u8;
        gap_adv_params.secondary_phy = BLE_GAP_PHY_AUTO as u8;
        gap_adv_params.interval = 400;

        let result = unsafe{sd_ble_gap_adv_set_configure(&mut adv_handle, &gap_adv_data, &gap_adv_params)};
        if result != NRF_SUCCESS{
            defmt::error!("adv set cfg result: {:?}", result);
        }

        let result = unsafe{sd_ble_gap_adv_start(adv_handle, 1)};
        if result != NRF_SUCCESS{
            defmt::error!("adv start result: {:?}", result);
        }
        loop {
            //defmt::info!("idle");
            cortex_m::asm::wfi();
        }
    }

    #[init(spawn=[tx_data])]
    fn init(ctx: init::Context) -> init::LateResources {
        defmt::info!("init");
        // Device specific peripherals
        let device: nrf52840_hal::pac::Peripherals = ctx.device;

        let p0 = Parts0::new(device.P0);
        let p1 = Parts1::new(device.P1);

        let status_led = p1.p1_10.into_push_pull_output(Level::Low).degrade();
        let switch_pin = p1.p1_02.into_pullup_input().degrade();

        // Feather pin D12
        let spi_cs = p0.p0_08.into_push_pull_output(Level::High).degrade();
        // Feather pin D11
        let rfm_irq_pin = p0.p0_06.into_pullup_input().degrade();
        // Feather pin D10
        let rfm_reset = p0.p0_27.into_push_pull_output(Level::High).degrade();

        // Configure RTC1 to trigger an interrupt after 1s.
        let mut rtc1 = Rtc::new(device.RTC1, 0).unwrap();
        rtc1.enable_counter();
        rtc1.set_compare(RtcCompareReg::Compare0, 32768).unwrap();
        rtc1.enable_interrupt(RtcInterrupt::Compare0, None);

        // Enable interrupt from rfm d0 interrupt pin and use switch pin.
        let gpiote = Gpiote::new(device.GPIOTE);
        let gpiote_port = gpiote.port();
        gpiote_port.input_pin(&rfm_irq_pin).high();
        gpiote_port.input_pin(&switch_pin).low();
        gpiote_port.enable_interrupt();

        // Configure and create the rfm95
        let config = RfmConfig {
            implicit_header_mode_on: false,
            bandwidth: Bandwidth::K125,
            coding_rate: CodingRate::Rate4_8,
            low_frequency_mode: false,
            rx_payload_crc_on: true,
            spreading_factor: SpreadingFactor::Twelve,
        };
        let delay_timer = Timer::new(device.TIMER3);

        let mut spim = create_spim(
            device.SPIM0,
            p0.p0_13.degrade(),
            p0.p0_15.degrade(),
            p0.p0_14.degrade(),
        );

        let rfm95 = RFM95::new(&mut spim, spi_cs, rfm_reset, config, delay_timer).unwrap();

        // Configure and create the statemachine for the rfm95
        let context = Context {
            rfm95,
            ppm_correction: None,
            rf_correction: None,
        };
        let mut statemachine = StateMachine::new(context);

        let mut sm_ctx = TempContext { spim: &mut spim };
        statemachine
            .process_event(&mut sm_ctx, Events::Initialize)
            .unwrap();

        ctx.spawn.tx_data().unwrap();

        init::LateResources {
            statemachine,
            rfm_irq_pin,
            status_led,
            switch_pin,
            gpiote,
            rtc1,
            spim,
        }
    }

    #[task(binds = RTC1, resources = [rtc1], priority=2, spawn=[])]
    fn on_rtc1(ctx: on_rtc1::Context) {
        ctx.resources.rtc1.reset_event(RtcInterrupt::Compare0);
        ctx.resources.rtc1.clear_counter();

        defmt::trace!("Interrupt from rtc1");
    }

    #[task(binds = GPIOTE, resources = [gpiote, rfm_irq_pin, switch_pin], priority=3, spawn=[handle_rfm_interrupt, handle_retry_interrupt])]
    fn on_gpiote(ctx: on_gpiote::Context) {
        if ctx.resources.gpiote.port().is_event_triggered() {
            defmt::trace!("Interrupt from port event");

            if ctx.resources.rfm_irq_pin.is_high().unwrap() {
                defmt::trace!("Interrupt from port event, irq pin high");
                ctx.spawn.handle_rfm_interrupt().unwrap();
            }

            if ctx.resources.switch_pin.is_low().unwrap() {
                defmt::trace!("Interrupt from port event, switch pin low");
                ctx.spawn.handle_retry_interrupt().unwrap();
            }
        }
        // Reset all events
        ctx.resources.gpiote.reset_events();
    }

    #[task(resources=[statemachine], spawn=[tx_complete, rx_complete])]
    fn handle_rfm_interrupt(ctx: handle_rfm_interrupt::Context) {
        defmt::trace!("handle_rfm_interrupt");
        match ctx.resources.statemachine.state() {
            States::Transmitting => {
                ctx.spawn.tx_complete().unwrap();
            }
            States::Receiving => {
                ctx.spawn.rx_complete().unwrap();
            }
            _ => {
                defmt::info!("in handle_rfm_interrupt with unexpected state")
            }
        }
    }

    #[task(resources=[], spawn=[tx_data])]
    fn handle_retry_interrupt(_ctx: handle_retry_interrupt::Context) {
        defmt::trace!("NOT handling_rfm_interrupt");
        //ctx.spawn.tx_data().unwrap();
    }

    #[task(resources=[statemachine, spim], capacity=4)]
    fn tx_data(ctx: tx_data::Context) {
        let mut sm_ctx = TempContext {
            spim: ctx.resources.spim,
        };
        ctx.resources
            .statemachine
            .process_event(&mut sm_ctx, Events::StartTransmitting)
            .unwrap();
    }

    #[task(resources=[spim, statemachine])]
    fn rx_data(ctx: rx_data::Context) {
        let mut sm_ctx = TempContext {
            spim: ctx.resources.spim,
        };

        ctx.resources
            .statemachine
            .process_event(&mut sm_ctx, Events::StartReceiving)
            .unwrap();
    }

    #[task(resources=[spim, statemachine], spawn=[rx_data])]
    fn tx_complete(ctx: tx_complete::Context) {
        let mut sm_ctx = TempContext {
            spim: ctx.resources.spim,
        };
        ctx.resources
            .statemachine
            .process_event(&mut sm_ctx, Events::TxCompleted)
            .unwrap();

        ctx.spawn.rx_data().unwrap();
    }

    #[task(resources=[spim, statemachine, status_led], spawn=[tx_data])]
    fn rx_complete(ctx: rx_complete::Context) {
        let mut sm_ctx = TempContext {
            spim: ctx.resources.spim,
        };
        let result = ctx
            .resources
            .statemachine
            .process_event(&mut sm_ctx, Events::RxCompleted);

        if result.is_ok() {
            toggle_status_led(ctx.resources.status_led);

            ctx.spawn.tx_data().unwrap();
        }
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    extern "C" {
        fn I2S();
    }
};

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
