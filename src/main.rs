#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
//use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
//use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::{iprintln, Peripherals};
use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f4::stm32f429;

use bamboo_core::*;
use bamboo_core::entry::*;

#[entry]
fn main() -> ! {
    asm::nop(); // To not have main optimize to abort in release mode, remove when you add code

    let entry = [1,2,3];

    let mut out = [0; MAX_ENTRY_SIZE];
    let mut out2 = [0; MAX_ENTRY_SIZE];
    let payload = [1,2,3,4,5];
    let secret_key_bytes = [ 
        197, 236, 75, 1, 28, 156, 231, 168, 
        29, 26, 12, 113, 0, 150, 235, 94, 
        140, 223, 220, 213, 102, 242, 213, 42, 
        128, 46, 137, 204, 44, 53, 206, 8
    ];

    let public_key_bytes = [
        221, 153, 125, 189, 92, 63, 192, 146, 
        29, 154, 178, 208, 108, 47, 58, 74, 
        149, 140, 115, 129, 117, 166, 223, 169, 
        171, 72, 94, 32, 190, 154, 67, 189
    ];

    let public = PublicKey::from_bytes(&public_key_bytes).unwrap();
    let secret = SecretKey::from_bytes(&secret_key_bytes).unwrap();
    let key_pair = Keypair{public, secret};

    let result1 = publish(&mut out, Some(&key_pair), 1, &payload, false, None, None, None);
    let entry1 = decode(&out).unwrap();
    let result2 = publish(&mut out2, Some(&key_pair), 1, &payload, false, Some(entry1.seq_num), Some(&out), Some(&out));
    let entry2 = decode(&out2).unwrap();

    let peripherals = stm32f429::Peripherals::take().unwrap();

    let rcc = &peripherals.RCC;
    rcc.ahb1enr.write(|r| r.gpiogen().enabled());

    let gpiog = &peripherals.GPIOG;
    gpiog.otyper.write(|r|r.ot13().push_pull());
    gpiog.moder.write(|r|r.moder13().output());

    loop {
        gpiog.odr.modify(|r, w| {
            let current = r.odr13().bit();
            w.odr13().bit(!current)
        });
        verify(&out2, Some(&payload), Some(&out), Some(&out)).unwrap();
        let mut p = Peripherals::take().unwrap();
        let stim = &mut p.ITM.stim[0];
        iprintln!(stim, "hello");
    }
}
