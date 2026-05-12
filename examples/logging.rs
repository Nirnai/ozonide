//! Minimal defmt test

#![no_std]
#![no_main]

use panic_probe as _;
use defmt_rtt as _;
use cortex_m_rt::entry;
use stm32h7xx_hal::pac;

#[entry]
fn main() -> ! {
    let _dp = pac::Peripherals::take().unwrap();
    
    cortex_m::asm::delay(16_000_000);  // Wait for RTT
    
    defmt::println!("Hello from defmt!");
    defmt::info!("This is an info message");
    defmt::warn!("This is a warning");
    defmt::error!("This is an error");
    
    let mut counter = 0u32;
    loop {
        counter += 1;
        if counter % 100_000 == 0 {
            defmt::info!("Counter: {}", counter / 100_000);
        }
    }
}
