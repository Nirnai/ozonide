use ozonide_core::topics::IMU_TOPIC;

use crate::imu_simulated::ImuSimulated;

#[embassy_executor::task]
pub async fn imu_task(imu: &'static mut ImuSimulated) {
    ozonide_core::tasks::imu_task(imu).await;
}

#[embassy_executor::task]
pub async fn imu_rate_monitor() {
    ozonide_core::tasks::rate_monitor("IMU", IMU_TOPIC.count()).await;
}
