//! Maximum Speed Example for WeAct STM32H743
//! 
//! Configures the STM32H743 to run at maximum speed (480 MHz) with VOS0 voltage scaling.
//! 
//! This example demonstrates:
//! - VOS0 voltage scaling configuration for maximum performance
//! - 480 MHz system clock from HSE (25 MHz crystal on WeAct board)
//! - Proper flash wait states for high-speed operation
//! - LED blink at max speed to verify operation
//! 
//! Expected behavior: LED blinks on/off every 250ms (faster than the 200MHz example)

#![no_std]
#![no_main]

use panic_probe as _;
use defmt_rtt as _;
use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    defmt::info!("=== Max Speed (480 MHz) Example Starting ===");
    
    // Get device peripherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    defmt::info!("Peripherals acquired");
    
    // Configure power domain with VOS0 for maximum performance (480 MHz)
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0(&dp.SYSCFG).freeze();
    
    defmt::info!("VOS0 voltage scaling enabled");
    
    // Configure clocks for 480 MHz operation
    // WeAct board has 25 MHz HSE crystal
    // PLL configuration: 25 MHz HSE -> /5 = 5 MHz -> x192 = 960 MHz VCO -> /2 = 480 MHz
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(480.MHz())  // Request 480 MHz system clock
        .hclk(240.MHz())    // AHB clock at 240 MHz (max for peripherals)
        .pll1_strategy(stm32h7xx_hal::rcc::PllConfigStrategy::Iterative)
        .freeze(pwrcfg, &dp.SYSCFG);
    
    // Log the actual achieved frequencies
    defmt::info!("Clock configuration complete:");
    defmt::info!("  System Clock (SYSCLK): {} MHz", ccdr.clocks.sys_ck().raw() / 1_000_000);
    defmt::info!("  AHB Clock (HCLK):      {} MHz", ccdr.clocks.hclk().raw() / 1_000_000);
    defmt::info!("  APB1 Clock:            {} MHz", ccdr.clocks.pclk1().raw() / 1_000_000);
    defmt::info!("  APB2 Clock:            {} MHz", ccdr.clocks.pclk2().raw() / 1_000_000);
    defmt::info!("  APB3 Clock:            {} MHz", ccdr.clocks.pclk3().raw() / 1_000_000);
    defmt::info!("  APB4 Clock:            {} MHz", ccdr.clocks.pclk4().raw() / 1_000_000);
    
    // Verify we achieved 480 MHz
    let sysclk_mhz = ccdr.clocks.sys_ck().raw() / 1_000_000;
    if sysclk_mhz == 480 {
        defmt::info!("✓ Successfully running at 480 MHz!");
    } else {
        defmt::warn!("⚠ Expected 480 MHz but got {} MHz", sysclk_mhz);
    }
    
    // Configure GPIO - PE3 as output (LED on WeAct board)
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let mut led = gpioe.pe3.into_push_pull_output();
    
    defmt::info!("LED configured on PE3");
    
    // Create delay provider using SysTick
    let mut delay = cp.SYST.delay(ccdr.clocks);
    
    defmt::info!("Starting fast blink loop at 480 MHz...");
    defmt::info!("LED blink period: 250ms (faster than 200 MHz example)");
    
    let mut counter = 0u32;
    loop {
        led.set_high();
        delay.delay_ms(250_u32);  // Faster blink to show we're running fast!
        
        led.set_low();
        delay.delay_ms(250_u32);
        
        counter += 1;
        if counter % 10 == 0 {
            defmt::debug!("Blink cycle: {} (running at {} MHz)", counter, sysclk_mhz);
        }
    }
}
