#![no_std]
#![no_main]

use bbqueue::{BBBuffer, Producer, Consumer, ConstBBBuffer, consts::*};
use nrf52840_hal as hal;
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
use nrf_bamboo_rs::ble::attrs;
use nrf_bamboo_rs::rfm_statemachine::*;

use rfm95_rs::{
    lora::{
        modem_config1::{Bandwidth, CodingRate},
        modem_config2::SpreadingFactor,
    },
    Config as RfmConfig, RFM95,
};
use rtic::app;

use core::sync::atomic::{compiler_fence, Ordering};
use rubble::{
    config::Config,
    l2cap::{BleChannelMap, L2CAPState},
    link::{
        ad_structure::AdStructure,
        queue::{PacketQueue, SimpleQueue},
        LinkLayer, Responder, MIN_PDU_BUF,
    },
    security::NoSecurity,
    time::{Duration, Timer as RubbleTimer},
};
use rubble_nrf5x::{
    radio::{BleRadio, PacketBuffer},
    timer::BleTimer,
    utils::get_device_address,
};

pub enum AppConfig {}

impl Config for AppConfig {
    type Timer = BleTimer<hal::pac::TIMER0>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<attrs::LoraMessagingAttrs, NoSecurity>;
    type PacketQueue = &'static mut SimpleQueue;
}
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

        #[init(BBBuffer(ConstBBBuffer::new()))]
        lora_tx_queue: BBBuffer<U512>,
        #[init(BBBuffer(ConstBBBuffer::new()))]
        lora_rx_queue: BBBuffer<U512>,

        lora_tx_producer: Producer<'static, U512>,
        lora_rx_consumer: Consumer<'static, U512>,

        // Below is stuff for rubble:
        #[init([0; MIN_PDU_BUF])]
        ble_tx_buf: PacketBuffer,
        #[init([0; MIN_PDU_BUF])]
        ble_rx_buf: PacketBuffer,
        #[init(SimpleQueue::new())]
        tx_queue: SimpleQueue,
        #[init(SimpleQueue::new())]
        rx_queue: SimpleQueue,
        ble_ll: LinkLayer<AppConfig>,
        ble_r: Responder<AppConfig>,
        radio: BleRadio,
    }

    #[idle(resources=[])]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            // Work around https://github.com/rust-lang/rust/issues/28728
            compiler_fence(Ordering::SeqCst);
            //defmt::info!("idle");
            cortex_m::asm::wfi();
        }
    }

    #[init(spawn=[rx_data], resources =[ble_tx_buf, ble_rx_buf, tx_queue, rx_queue, lora_rx_queue, lora_tx_queue])]
    fn init(ctx: init::Context) -> init::LateResources {
        defmt::info!("init");

        // Device specific peripherals
        let device: nrf52840_hal::pac::Peripherals = ctx.device;

        let p0 = Parts0::new(device.P0);
        let p1 = Parts1::new(device.P1);

        // On reset, the internal high frequency clock is already used, but we
        // also need to switch to the external HF oscillator. This is needed
        // for Bluetooth to work.
        let _clocks = hal::clocks::Clocks::new(device.CLOCK)
            .enable_ext_hfosc()
            .set_lfclk_src_rc()
            .start_lfclk();

        let ble_timer = BleTimer::init(device.TIMER0);

        // Determine device address
        let device_address = get_device_address();

        let mut radio = BleRadio::new(
            device.RADIO,
            &device.FICR,
            ctx.resources.ble_tx_buf,
            ctx.resources.ble_rx_buf,
        );

        // Feather pin D13
        let other_status_led = p1.p1_09.into_push_pull_output(Level::Low).degrade();

        // Create the actual BLE stack objects
        let mut ble_ll = LinkLayer::<AppConfig>::new(device_address, ble_timer);

        // Create TX/RX queues
        let (tx, tx_cons) = ctx.resources.tx_queue.split();
        let (rx_prod, rx) = ctx.resources.rx_queue.split();

        let ble_r = Responder::new(
            tx,
            rx,
            L2CAPState::new(BleChannelMap::with_attributes(
                attrs::LoraMessagingAttrs::new(other_status_led),
            )),
        );

        // Send advertisement and set up regular interrupt
        let next_update = ble_ll
            .start_advertise(
                Duration::from_millis(200),
                &[AdStructure::CompleteLocalName("Lora Messaging Service")],
                &mut radio,
                tx_cons,
                rx_prod,
            )
            .unwrap();

        ble_ll.timer().configure_interrupt(next_update);

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
        let (lora_tx_producer, lora_tx_consumer) = ctx.resources.lora_tx_queue.try_split().unwrap();
        let (lora_rx_producer, lora_rx_consumer) = ctx.resources.lora_rx_queue.try_split().unwrap();

        let context = Context {
            rfm95,
            ppm_correction: None,
            rf_correction: None,
            rx_producer: lora_rx_producer,
            tx_consumer: lora_tx_consumer
        };
        let mut statemachine = StateMachine::new(context);

        let mut sm_ctx = TempContext { spim: &mut spim };
        statemachine
            .process_event(&mut sm_ctx, Events::Initialize)
            .unwrap();

        ctx.spawn.rx_data().unwrap();

        init::LateResources {
            lora_tx_producer,
            lora_rx_consumer,
            statemachine,
            rfm_irq_pin,
            status_led,
            switch_pin,
            ble_ll,
            gpiote,
            radio,
            ble_r,
            rtc1,
            spim,
        }
    }

    #[task(binds = RADIO, resources = [radio, ble_ll], spawn = [ble_worker], priority = 3)]
    fn radio(ctx: radio::Context) {
        let ble_ll: &mut LinkLayer<AppConfig> = ctx.resources.ble_ll;
        if let Some(cmd) = ctx
            .resources
            .radio
            .recv_interrupt(ble_ll.timer().now(), ble_ll)
        {
            ctx.resources.radio.configure_receiver(cmd.radio);
            ble_ll.timer().configure_interrupt(cmd.next_update);

            if cmd.queued_work {
                // If there's any lower-priority work to be done, ensure that happens.
                // If we fail to spawn the task, it's already scheduled.
                ctx.spawn.ble_worker().ok();
            }
        }
    }

    #[task(binds = TIMER0, resources = [radio, ble_ll], spawn = [ble_worker], priority = 3)]
    fn timer0(ctx: timer0::Context) {

        let timer = ctx.resources.ble_ll.timer();
        if !timer.is_interrupt_pending() {
            return;
        }
        timer.clear_interrupt();

        let cmd = ctx.resources.ble_ll.update_timer(ctx.resources.radio);
        ctx.resources.radio.configure_receiver(cmd.radio);

        ctx.resources
            .ble_ll
            .timer()
            .configure_interrupt(cmd.next_update);

        if cmd.queued_work {
            // If there's any lower-priority work to be done, ensure that happens.
            // If we fail to spawn the task, it's already scheduled.
            ctx.spawn.ble_worker().ok();
        }
    }

    #[task(resources = [ble_r], priority = 2, capacity = 10)]
    fn ble_worker(ctx: ble_worker::Context) {
        // Fully drain the packet queue
        while ctx.resources.ble_r.has_work() {
            ctx.resources.ble_r.process_one().unwrap();
        }
    }
    #[task(binds = RTC1, resources = [rtc1, ble_ll], priority=2, spawn=[])]
    fn on_rtc1(mut ctx: on_rtc1::Context) {
        ctx.resources.rtc1.reset_event(RtcInterrupt::Compare0);
        ctx.resources.rtc1.clear_counter();

        ctx.resources.ble_ll.lock(|ble_ll|{
            if !ble_ll.is_connected() && !ble_ll.is_advertising() {
                defmt::info!("ble not connected and not advertising");
                // For now just reset?
            }
        });

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
            .ok();
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

    #[task(resources=[spim, statemachine, lora_rx_consumer, status_led], spawn=[tx_data])]
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

            let mut grant = ctx.resources.lora_rx_consumer.split_read().unwrap();
            grant.to_release(grant.combined_len());
            let (buf_a, buf_b) = grant.bufs();
            
            defmt::info!("received lora bytes: {:?}, {:?}", buf_a, buf_b);

            ctx.spawn.tx_data().unwrap();
        }
    }

    // RTIC requires that unused interrupts are declared in an extern block when
    // using software tasks; these free interrupts will be used to dispatch the
    // software tasks.
    extern "C" {
        fn I2S();
        fn WDT();
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
