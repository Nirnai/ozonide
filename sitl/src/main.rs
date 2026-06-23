use embassy_executor::Spawner;
use static_cell::StaticCell;

use ozonide_core::control::indi::CascadedController;
use ozonide_core::estimation::PassthroughStateEstimator;

mod actuator_simulated;
mod actuator_telemetry_simulated;
mod ground_truth_simulated;
mod imu_simulated;
mod setpoint_simulated;
mod tasks;

use actuator_simulated::ActuatorSimulated;
use actuator_telemetry_simulated::ActuatorTelemetrySimulated;
use ground_truth_simulated::GroundTruthSimulated;
use imu_simulated::ImuSimulated;
use setpoint_simulated::SetpointSimulated;

static IMU: StaticCell<ImuSimulated> = StaticCell::new();
static ACTUATOR_TELEMETRY: StaticCell<ActuatorTelemetrySimulated> = StaticCell::new();
static GROUND_TRUTH: StaticCell<GroundTruthSimulated> = StaticCell::new();
static STATE_ESTIMATOR: StaticCell<PassthroughStateEstimator> = StaticCell::new();
static CONTROLLER: StaticCell<CascadedController> = StaticCell::new();
static SETPOINT_SOURCE: StaticCell<SetpointSimulated> = StaticCell::new();
static ACTUATOR: StaticCell<ActuatorSimulated> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    log::info!("Ozonide SITL starting...");

    let imu = IMU.init(ImuSimulated::new());
    spawner.spawn(tasks::imu_task(imu).unwrap());

    let actuator_telemetry = ACTUATOR_TELEMETRY.init(ActuatorTelemetrySimulated::new());
    spawner.spawn(tasks::actuator_telemetry_task(actuator_telemetry).unwrap());

    let ground_truth = GROUND_TRUTH.init(GroundTruthSimulated::new());
    spawner.spawn(tasks::ground_truth_task(ground_truth).unwrap());

    let state_estimator = STATE_ESTIMATOR.init(PassthroughStateEstimator::new());
    spawner.spawn(tasks::state_estimation_task(state_estimator).unwrap());

    let controller = CONTROLLER.init(tasks::make_controller());
    let setpoint_source = SETPOINT_SOURCE.init(tasks::make_setpoint_source());
    spawner.spawn(tasks::control_task(controller, setpoint_source).unwrap());

    let actuator = ACTUATOR.init(ActuatorSimulated::new());
    spawner.spawn(tasks::actuator_task(actuator).unwrap());

    spawner.spawn(tasks::imu_rate_monitor().unwrap());
}
