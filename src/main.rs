#![no_std]
#![no_main]

mod board;
mod drivers;
mod algorithms;
mod tasks;
mod utils;
mod vehicle;
mod config;
mod messaging;

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;
#[entry]
fn main() -> ! {
    defmt::info!("Ozonide flight controller starting...");
   
    let mut vehicle = vehicle::Vehicle::init();
    vehicle.init_sensors();

    loop {
        cortex_m::asm::wfi();
    }
}