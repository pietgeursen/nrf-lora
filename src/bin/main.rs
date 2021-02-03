#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nrf52840_hal::pac::Peripherals;
use nrf_bamboo_rs as _;
use nrf52840_hal::gpio::p0::Parts;
use nrf52840_hal::twim::Pins as TwimPins;
use nrf52840_hal::Twim;
use nrf52840_hal::pac::twim0::frequency::FREQUENCY_A;
use nrf_softdevice_s132::*;

use bamboo_rs_core::entry::*;
use bamboo_rs_core::*;
use bmp280::*;
use apds9960::*;

#[entry]
fn main() -> ! {
    let periph = Peripherals::take().unwrap();
    periph.P1.dir.write(|w| w.pin10().output());

//    let p0 = Parts::new(periph.P0);
//    let scl_pin = p0.p0_11.into_floating_input();
//    let sda_pin = p0.p0_12.into_floating_input();
//    let twim_pins = TwimPins{ scl: scl_pin.degrade(), sda: sda_pin.degrade() };
//    let twim = Twim::new(periph.TWIM0, twim_pins, FREQUENCY_A::K100);
//
//    let bmp_config = Config::handheld_device_lowpower();
//    let mut bmp280 = BMP280::new(twim, I2CAddress::SdoPulledUp, bmp_config )
//        .unwrap()
//        .into_normal_mode()
//        .unwrap();

    let p0 = Parts::new(periph.P0);
    let scl_pin = p0.p0_11.into_floating_input();
    let sda_pin = p0.p0_12.into_floating_input();
    let twim_pins = TwimPins{ scl: scl_pin.degrade(), sda: sda_pin.degrade() };
    let twim = Twim::new(periph.TWIM0, twim_pins, FREQUENCY_A::K100);

    let mut apds9960 = Apds9960::new(twim);
    apds9960.enable().unwrap();
    apds9960.enable_light().unwrap();

    let mut is_enabled = 0;
    let err_code = unsafe{ sd_softdevice_is_enabled(&mut is_enabled)};
    defmt::info!("is_enabled: {:?}", is_enabled);
    defmt::info!("err code: {:?}", err_code);

    defmt::info!("hellllloooo");
    let mut out = [0; MAX_ENTRY_SIZE];
    let mut out2 = [0; MAX_ENTRY_SIZE];
    let payload = [1, 2, 3, 4, 5];

    let secret_key_bytes = [
        197, 236, 75, 1, 28, 156, 231, 168, 29, 26, 12, 113, 0, 150, 235, 94, 140, 223, 220, 213,
        102, 242, 213, 42, 128, 46, 137, 204, 44, 53, 206, 8,
    ];

    let public_key_bytes = [
        221, 153, 125, 189, 92, 63, 192, 146, 29, 154, 178, 208, 108, 47, 58, 74, 149, 140, 115,
        129, 117, 166, 223, 169, 171, 72, 94, 32, 190, 154, 67, 189,
    ];

    let public = PublicKey::from_bytes(&public_key_bytes).unwrap();
    let secret = SecretKey::from_bytes(&secret_key_bytes).unwrap();
    let key_pair = Keypair { public, secret };

    let result1 = publish(
        &mut out,
        &key_pair,
        1,
        &payload,
        false,
        None,
        None,
        None,
    );
    let entry1 = decode(&out).unwrap();
    defmt::info!("published first entry result was ok: {:?}", result1.is_ok());
    defmt::info!("published first entry was: {:?}", entry1.seq_num as u8);

    let result2 = publish(
        &mut out2,
        &key_pair,
        1,
        &payload,
        false,
        Some(entry1.seq_num),
        Some(&out),
        Some(&out),
    );
    let entry2 = decode(&out2).unwrap();

    defmt::info!("published second entry result was: {:?}", result2.is_ok());
    defmt::info!("published first entry was: {:?}", entry2.seq_num as u8);

    loop {
        let result = verify(&out2, Some(&payload), Some(&out), Some(&out)).is_ok();
        assert!(result);
        periph.P1.out.modify(|r, w| {
            let current = r.pin10().bit();
            w.pin10().bit(!current)
        });

        apds9960.read_light().unwrap();
        //let temp = bmp280.read_temperature().unwrap();
        //defmt::info!("temp was: {:?}", temp);

        //let pressure = bmp280.read_pressure().unwrap();
        //defmt::info!("pressure was: {:?}", pressure);
    }
}
