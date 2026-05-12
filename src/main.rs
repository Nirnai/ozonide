#![no_std]
#![no_main]

// Declare all modules
mod algorithms;
mod board;
mod drivers;
mod tasks;
mod types;
mod utils;

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    defmt::info!("Ozonide flight controller starting...");

    // Get device peripherals (required for interrupt vector table)
    let _core_peripherals = cortex_m::Peripherals::take().unwrap();
    let device_peripherals = pac::Peripherals::take().unwrap();

    // Configure power and clocks
    let power_control: stm32h7xx_hal::pwr::Pwr = device_peripherals.PWR.constrain();
    let power_configuration = power_control.vos0(&device_peripherals.SYSCFG).freeze();

    let clock_control = device_peripherals.RCC.constrain();
    let clock_configuration = clock_control
        .sys_ck(480.MHz())
        .hclk(240.MHz())
        .pll1_strategy(stm32h7xx_hal::rcc::PllConfigStrategy::Iterative)
        .freeze(power_configuration, &device_peripherals.SYSCFG);
    defmt::info!("Clock configuration complete:");
    defmt::info!("  System Clock (SYSCLK): {} MHz", clock_configuration.clocks.sys_ck().raw() / 1_000_000);
    defmt::info!("  AHB Clock (HCLK):      {} MHz", clock_configuration.clocks.hclk().raw() / 1_000_000);
    defmt::info!("  APB1 Clock:            {} MHz", clock_configuration.clocks.pclk1().raw() / 1_000_000);
    defmt::info!("  APB2 Clock:            {} MHz", clock_configuration.clocks.pclk2().raw() / 1_000_000);
    defmt::info!("  APB3 Clock:            {} MHz", clock_configuration.clocks.pclk3().raw() / 1_000_000);
    defmt::info!("  APB4 Clock:            {} MHz", clock_configuration.clocks.pclk4().raw() / 1_000_000);


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
