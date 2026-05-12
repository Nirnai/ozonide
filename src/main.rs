#![no_std]
#![no_main]

// Declare all modules
mod types;
mod drivers;
mod algorithms;
mod tasks;
mod utils;
mod board;

use panic_probe as _;
use defmt_rtt as _;
use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    defmt::info!("Ozonide flight controller starting...");
    
    // Get device peripherals (required for interrupt vector table)
    let dp = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();
    
    // Configure power and clocks
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    
    let rcc = dp.RCC.constrain();
    let _ccdr = rcc.sys_ck(480.MHz()).freeze(pwrcfg, &dp.SYSCFG);
    
    defmt::info!("Hardware initialized - STM32H743 @ 480MHz");
    
    // TODO: Initialize board peripherals
    // let board = board::Board::init();
    
    // TODO: Initialize tasks
    // let mut control_loop = tasks::control_loop::ControlLoop::new();
    // let mut sensor_fusion = tasks::sensor_fusion::SensorFusion::new();
    
    loop {
        // TODO: Main loop
        // 1. Read sensors
        // 2. Update state estimate
        // 3. Compute control
        // 4. Output to ESCs
        
        cortex_m::asm::nop(); // Prevent optimization removing the loop
    }
}
