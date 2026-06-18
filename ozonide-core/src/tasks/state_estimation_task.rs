use crate::topics::{IMU_TOPIC, ACTUATOR_TELEMETRY_TOPIC, VEHICLE_STATE_TOPIC};
use crate::traits::{SensorData, StateEstimator};

pub async fn state_estimation_task(state_estimator: &mut impl StateEstimator) {
    let mut imu_subscriber = IMU_TOPIC.subscriber();
    let mut actuator_observer = ACTUATOR_TELEMETRY_TOPIC.observer();

    let state_publisher = VEHICLE_STATE_TOPIC.publisher();

    loop {
        let imu_data = imu_subscriber.changed().await;
        let motor_data = actuator_observer.try_get();
        let sensor_data = SensorData {
            imu: Some(&imu_data),
            actuator: motor_data.as_ref(),
            battery: None,
        };
        let estimated_state = state_estimator.update(&sensor_data);
        state_publisher.publish(estimated_state);
    }
}