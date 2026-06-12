use crate::topics::{IMU_TOPIC, VEHICLE_STATE_TOPIC};
use crate::traits::{StateEstimator, SensorData};

pub async fn state_estimation_task(state_estimator: &mut impl StateEstimator){  
    let mut imu_subscriber = IMU_TOPIC.subscriber(); 
    // TODO: Add more sensors when supported

    let state_publisher = VEHICLE_STATE_TOPIC.publisher();

    loop {
        let imu_data = imu_subscriber.changed().await;
        let sensor_data = SensorData{imu: Some(&imu_data), motor: None, battery: None};
        let estimated_state = state_estimator.update(&sensor_data);
        state_publisher.publish(estimated_state);
    }
}