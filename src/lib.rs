#![no_std]

use core::sync::atomic::{AtomicUsize, Ordering};

// pick a panicking behavior
//use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
//use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
//use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use defmt_rtt as _; // global logger
use nrf52840_hal as _;
use panic_probe as _;

#[defmt::timestamp]
fn timestamp() -> u64 {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n as u64
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
