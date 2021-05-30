#![no_std]
#![no_main]

use nrf52840_hal::gpio::{Disconnected, Level, Output, Pin, PushPull};
use nrf52840_hal::pac::{DWT, SPIM0};
use nrf52840_hal::prelude::*;
use nrf52840_hal::spim::{
    Frequency as SpimFrequency, Mode as SpimMode, Phase, Pins as SpimPins, Polarity,
};
use nrf52840_hal::Spim;
use nrf52840_hal::Timer;
use nrf52840_hal::{gpio::p0::Parts as Parts0, gpio::p1::Parts as Parts1, gpiote::*};
use nrf_bamboo_rs as _;
use nrf_bamboo_rs::rfm_statemachine::*;

use rfm95_rs::{
    lora::{
        modem_config1::{Bandwidth, CodingRate},
        modem_config2::SpreadingFactor,
    },
    Config as RfmConfig, RFM95,
};
use rtic::app;

use rtic::cyccnt::U32Ext as _;
const PERIOD: u32 = 128_000_000;


#[app(device = nrf52840_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        status_led: Pin<Output<PushPull>>,
        gpiote: Gpiote,
        spim: Spim<SPIM0>,
        statemachine: StateMachine<Context>,
    }

    #[idle(resources=[spim])]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            //defmt::info!("idle");
            //cortex_m::asm::wfi();
        }
    }

    #[init(spawn=[tx_data])]
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
            p0.p0_15.degrade(),
            p0.p0_14.degrade(),
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

        let rfm95 = RFM95::new(&mut spim, spi_cs, rfm_reset, config, delay_timer).unwrap();

        let context = Context {
            rfm95,
            ppm_correction: None,
            rf_correction: None,
        };
        let mut statemachine = StateMachine::new(context);

        let mut sm_ctx = TempContext {
            spim: &mut spim,
        };
        statemachine.process_event(&mut sm_ctx, Events::Initialize).unwrap();

        ctx.spawn.tx_data().unwrap();

        init::LateResources {
            statemachine,
            status_led,
            gpiote,
            spim,
        }
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

    #[task(schedule=[tx_data], resources=[spim, statemachine, status_led], spawn=[tx_data])]
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

            ctx.schedule
                .tx_data(ctx.scheduled + PERIOD.cycles())
                .unwrap();
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

fn configure_systimer(core: &mut rtic::Peripherals) {
    core.DCB.enable_trace();
    DWT::unlock();
    core.DWT.enable_cycle_counter();
}
