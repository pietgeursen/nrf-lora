#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nrf52832_hal::pac::Peripherals;
use nrf_bamboo_rs as _;

use bamboo_core::entry::*;
use bamboo_core::*;

#[entry]
fn main() -> ! {
    let periph = Peripherals::take().unwrap();
    periph.P0.dir.write(|w| w.pin17().output());

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
        Some(&key_pair),
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
        Some(&key_pair),
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
        let result = verify(&out2, Some(&payload), Some(&out), Some(&out)).unwrap();
        assert!(result);
        periph.P0.out.modify(|r, w| {
            let current = r.pin17().bit();
            w.pin17().bit(!current)
        });
    }
}
