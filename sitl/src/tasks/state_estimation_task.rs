use ozonide_core::estimation::ComplementaryAttitudeEstimator;

#[embassy_executor::task]
pub async fn attitude_estimation_task(
    state_estimator: &'static mut ComplementaryAttitudeEstimator,
) {
    ozonide_core::tasks::state_estimation_task(state_estimator).await;
}
