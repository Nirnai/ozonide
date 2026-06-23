// TODO: Replace the injected `StateEstimator` with a UKF. It should fuse IMU
// (gyro + accel), actuator feedback (motor Ω via ACTUATOR_TELEMETRY_TOPIC), and
// camera/VIO measurements into a single consistent estimate of position,
// velocity, attitude, and gyro bias. Today a `PassthroughStateEstimator` stands
// in: it forwards SITL ground-truth pose and raw IMU rates. The UKF drops into
// the same slot — add a camera measurement to `SensorData`, implement the UKF as
// a `StateEstimator`, and the `GROUND_TRUTH_STATE_TOPIC` plumbing here goes away.

use crate::topics::{ACTUATOR_TELEMETRY_TOPIC, GROUND_TRUTH_STATE_TOPIC, IMU_TOPIC, VEHICLE_STATE_TOPIC};
use crate::traits::{SensorData, StateEstimator};

pub async fn state_estimation_task(state_estimator: &mut impl StateEstimator) {
    let mut imu_subscriber = IMU_TOPIC.subscriber();
    let mut actuator_observer = ACTUATOR_TELEMETRY_TOPIC.observer();
    let mut ground_truth_observer = GROUND_TRUTH_STATE_TOPIC.observer();

    let state_publisher = VEHICLE_STATE_TOPIC.publisher();

    loop {
        let imu_data = imu_subscriber.changed().await;
        let motor_data = actuator_observer.try_get();
        let ground_truth = ground_truth_observer.try_get();
        let sensor_data = SensorData {
            imu: Some(&imu_data),
            actuator: motor_data.as_ref(),
            battery: None,
            ground_truth: ground_truth.as_ref(),
        };
        let state = state_estimator.update(&sensor_data);
        state_publisher.publish(state);
    }
}