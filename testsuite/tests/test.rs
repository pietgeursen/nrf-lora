#![no_std]
#![no_main]

use bamboo_core::*;
use cortex_m_rt::entry;
use nrf_bamboo_rs as _;

use nrf52832_hal::{
    pac::{Peripherals, TEMP},
    Temp,
};

fn test_temp(temp: TEMP) {
    defmt::info!("testing temperature sensor...");
    let mut temp = Temp::new(temp);
    let reading = temp.measure();
    defmt::info!("temp reading was {:?}", reading.to_num::<u8>());

    assert!(reading > -10, "temp = {}°C. that's too cold!", reading);
    assert!(reading < 50, "temp = {}°C. that's too hot!", reading);
}

fn verify_valid_entry() {
    defmt::info!("testing verify valid entry...");
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

    let payload = "hello bamboo!";
    let mut out = [0u8; 512];

    let size = publish(
        &mut out,
        Some(&key_pair),
        0,
        payload.as_bytes(),
        false,
        None,
        None,
        None,
    )
    .unwrap();

    let mut entry = decode(&out[..size]).unwrap();

    assert!(entry.verify_signature().unwrap());
}
fn verify_invalid_entry() {
    defmt::info!("testing verify invalid entry...");
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

    let payload = "hello bamboo!";
    let mut out = [0u8; 512];

    let size = publish(
        &mut out,
        Some(&key_pair),
        0,
        payload.as_bytes(),
        false,
        None,
        None,
        None,
    )
    .unwrap();

    out[30] = !out[30];
    let mut entry = decode(&out[..size]).unwrap();

    assert!(entry.verify_signature().is_err());
}

#[entry]
fn main() -> ! {
    let periph = Peripherals::take().unwrap();

    defmt::info!("running on-device tests...");

    test_temp(periph.TEMP);
    verify_valid_entry();
    verify_invalid_entry();

    nrf_bamboo_rs::exit();
}
