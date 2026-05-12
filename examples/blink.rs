//! LED Blink Example for WeAct STM32H743
//! 
//! Blinks the onboard LED to verify basic hardware functionality.
//! 
//! Expected behavior: LED blinks on/off every 500ms
//! 
//! NOTE: Check your board schematic for the correct LED pin!
//! Common WeAct H743 LED pins: PE3 (green), PA1

#![no_std]
#![no_main]

use panic_probe as _;
use defmt_rtt as _;
use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    defmt::info!("=== LED Blink Example Starting ===");
    
    // Get device peripherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    defmt::info!("Peripherals acquired");
    
    // Configure power and clocks
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    
    // Use 200 MHz - safe frequency that works with default voltage scaling
    // (480 MHz requires VOS0 voltage scaling which needs more configuration)
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz())  // System clock @ 200MHz - conservative and reliable
        .freeze(pwrcfg, &dp.SYSCFG);
    
    defmt::info!("Clock configured: {} MHz", ccdr.clocks.sys_ck().raw() / 1_000_000);
    
    // Configure GPIO - PE3 as output (common LED pin on WeAct boards)
    // If your LED is on a different pin, change gpioe to the correct port
    // and pe3 to the correct pin number
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let mut led = gpioe.pe3.into_push_pull_output();
    
    defmt::info!("LED configured on PE3");
    
    // Create delay provider using SysTick
    let mut delay = cp.SYST.delay(ccdr.clocks);
    
    defmt::info!("Starting blink loop...");
    
    let mut counter = 0u32;
    loop {
        led.set_high();
        delay.delay_ms(500_u32);
        
        led.set_low();
        delay.delay_ms(500_u32);
        
        counter += 1;
        if counter % 10 == 0 {
            defmt::info!("Blink cycle: {}", counter);
        }
    }
}
