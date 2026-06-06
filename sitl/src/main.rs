use embassy_executor::Spawner;
use static_cell::StaticCell;

use ozonide_core::estimation::ComplementaryAttitudeEstimator;

mod imu_simulated;
mod actuator_simulated;
mod tasks;

use imu_simulated::ImuSimulated;
use actuator_simulated::ActuatorSimulated;

static IMU: StaticCell<ImuSimulated> = StaticCell::new();
static ATTITUDE_ESTIMATOR: StaticCell<ComplementaryAttitudeEstimator> = StaticCell::new();
static ACTUATOR: StaticCell<ActuatorSimulated> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    log::info!("Ozonide SITL starting...");
    let imu = IMU.init(ImuSimulated::new());
    spawner.spawn(tasks::imu_task(imu).unwrap());

    let attitude_estimator = ATTITUDE_ESTIMATOR.init(ComplementaryAttitudeEstimator::new(0.98));
    spawner.spawn(tasks::attitude_estimation_task(attitude_estimator).unwrap());

    // TODO: Spawn Control task

    let actuator = ACTUATOR.init(ActuatorSimulated::new());
    spawner.spawn(tasks::actuator_task(actuator).unwrap());
    
    spawner.spawn(tasks::imu_rate_monitor().unwrap());
}
