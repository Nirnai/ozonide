use embassy_executor::Spawner;
use static_cell::StaticCell;

mod imu_simulated;
mod tasks;

use imu_simulated::ImuSimulated;

static IMU: StaticCell<ImuSimulated> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    log::info!("Ozonide SITL starting...");
    let imu = IMU.init(ImuSimulated::new());
    spawner.spawn(tasks::imu_task(imu).unwrap());
    spawner.spawn(tasks::imu_rate_monitor().unwrap());
}
