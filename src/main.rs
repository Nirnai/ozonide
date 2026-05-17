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

#[entry]
fn main() -> ! {
    defmt::info!("Ozonide flight controller starting...");

    let board = board::Stm32h743vit6::init();

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
