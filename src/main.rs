#![no_std]
#![no_main]

mod board;
mod drivers;
mod algorithms;
mod tasks;
mod types;
mod utils;
mod vehicle;
mod config;

// Needed to call read
use crate::drivers::imu::Imu;

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;

#[entry]
fn main() -> ! {
    defmt::info!("Ozonide flight controller starting...");

    let mut vehicle = vehicle::Vehicle::from_config();


    // TODO: Initialize tasks
    // let mut control_loop = tasks::control_loop::ControlLoop::new();
    // let mut sensor_fusion = tasks::sensor_fusion::SensorFusion::new();

    loop {

        if let Some(imu) = &mut vehicle.imu {
            let data: types::ImuData = imu.read();
            defmt::info!("accel: ({}, {}, {})",
                data.linear_acceleration.x,
                data.linear_acceleration.y,
                data.linear_acceleration.z);
        }
        // TODO: Main loop
        // 1. Read sensors
        // 2. Update state estimate
        // 3. Compute control
        // 4. Output to ESCs

        cortex_m::asm::nop(); // Prevent optimization removing the loop
    }
}
