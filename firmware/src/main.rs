#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

mod board;
mod drivers;
mod executors;
mod tasks;

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Ozonide flight controller starting...");
    let board = board::Board::init();

    let urgent_tasks = executors::high_priority_executor();
    urgent_tasks.spawn(tasks::imu_task(board.imu).unwrap());

    executors::low_priority_executor(|spawner| {
        spawner.spawn(tasks::imu_rate_monitor().unwrap());
    });
}
