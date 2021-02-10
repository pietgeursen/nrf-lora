#![no_std]
#![no_main]

use nrf52840_hal::gpio::p0::Parts;
use nrf52840_hal::pac::twim0::frequency::FREQUENCY_A;
use nrf52840_hal::pac::P1;
use nrf52840_hal::pac::{DWT, TWIM0, SPIM0};
use nrf52840_hal::twim::Pins as TwimPins;
use nrf52840_hal::spim::{Pins as SpimPins, Mode as SpimMode, Polarity, Phase, Frequency as SpimFrequency};
use nrf52840_hal::{Twim, Spim};
use nrf52840_hal::gpio::Level;
use nrf_bamboo_rs as _;
use nrf_softdevice_s140::*;
use rtic::app;

use apds9960::*;
//use bamboo_rs_core::entry::*;
//use bamboo_rs_core::*;
use bmp280_rs::*;

use rtic::cyccnt::U32Ext as _;
const PERIOD: u32 = 64_000_000;

enum DisableableSpim{
    enabled(Spim<SPIM0>),
    disabled(SPIM0)
}

#[app(device = nrf52840_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        twim: Twim<TWIM0>,
        spim: Spim<SPIM0>,
        bmp280: BMP280<Twim<TWIM0>, ModeNormal>,
        port_one: P1,
    }

    #[init(schedule = [measure_bmp_pressure])]
    fn init(ctx: init::Context) -> init::LateResources {
        // Cortex-M peripherals
        defmt::info!("init");
        let mut core = ctx.core;
        // Device specific peripherals
        let mut  device: nrf52840_hal::pac::Peripherals = ctx.device;
        device.P1.dir.write(|w| w.pin10().output());
        toggle_status_led(&mut device.P1);

        // Initialize (enable) the monotonic timer (CYCCNT)
        defmt::info!("configure sys timer");
        configure_systimer(&mut core);

        // semantically, the monotonic timer is frozen at time "zero" during `init`
        // NOTE do *not* call `Instant::now` in this context; it will return a nonsense value
        defmt::info!("schedule task");
        ctx.schedule
            .measure_bmp_pressure(ctx.start + PERIOD.cycles())
            .unwrap();

        // Setup i2c pins and peripheral
        let p0 = Parts::new(device.P0);
        //let mut twim = create_twim0(&mut p0, device.TWIM0);

        let scl_pin = p0.p0_11.into_floating_input();
        let sda_pin = p0.p0_12.into_floating_input();
        let twim_pins = TwimPins {
            scl: scl_pin.degrade(),
            sda: sda_pin.degrade(),
        };
        let mut twim = Twim::new(device.TWIM0, twim_pins, FREQUENCY_A::K100);
        let bmp280 = create_bmp280(&mut twim);
        twim.disable();

        let mosi_pin = p0.p0_13.into_push_pull_output(Level::High);
        let miso_pin = p0.p0_15.into_floating_input();
        let sck_pin = p0.p0_14.into_push_pull_output(Level::High);

        let spim_pins = SpimPins {
            sck: sck_pin.degrade(),
            mosi: Some(mosi_pin.degrade()),
            miso: Some(miso_pin.degrade())
        };
        let mode = SpimMode{
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition
        };
        defmt::info!("create spim");
        let mut spim = Spim::new(device.SPIM0, spim_pins, SpimFrequency::K500, mode, 0xFF);
        defmt::info!("created spim");

        let hello = b"hello world";
        let mut buffer = [0u8; 11];
        buffer.copy_from_slice(&hello[..]);
        let mut spi_cs = p0.p0_08.into_push_pull_output(Level::High).degrade();

        let result = spim.write(&mut spi_cs, &buffer[..]);
        defmt::info!("spi result is ok {:?}", result.is_ok());

        defmt::info!("created late resources");
        init::LateResources {
            twim,
            port_one: device.P1,
            bmp280,
            spim
        }
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {}
    }

    #[task(schedule = [measure_bmp_pressure], resources = [port_one, twim, bmp280])]
    fn measure_bmp_pressure(ctx: measure_bmp_pressure::Context) {
        defmt::info!("measure_bmp_pressure");
        let mut twim = ctx.resources.twim;
        twim.enable();

        let pressure = ctx.resources.bmp280.read_pressure(&mut twim);
        defmt::info!("pressure: {:?}", pressure);
        if let Ok(p) = pressure {
            defmt::info!("pressure: {:?} pa", p / 256);
        }

        let temperature = ctx.resources.bmp280.read_temperature(&mut twim);
        twim.disable();
        defmt::info!("temperature: {:?}", temperature);

        toggle_status_led(ctx.resources.port_one);

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

fn toggle_status_led(port_one: &mut nrf52840_hal::pac::P1) {
    port_one.out.modify(|r, w| {
        let current = r.pin10().bit();
        w.pin10().bit(!current)
    });
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
//fn create_spim0(p0: &mut Parts, spim0: SPIM0) -> Spim<SPIM0> {
//    let mosi_pin = p0.p0_13.into_push_pull_output(Level::High);
//    let miso_pin = p0.p0_12.into_floating_input();
//    let sck_pin = p0.p0_15.into_push_pull_output(Level::High);
//
//    let spim_pins = SpimPins {
//        sck: sck_pin.degrade(),
//        mosi: Some(mosi_pin.degrade()),
//        miso: Some(miso_pin.degrade())
//    };
//    let mode = SpimMode{
//        polarity: Polarity::IdleLow,
//        phase: Phase::CaptureOnFirstTransition
//
//    };
//    Spim::new(spim0, spim_pins, SpimFrequency::K500, mode, 0x00)
//}
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
