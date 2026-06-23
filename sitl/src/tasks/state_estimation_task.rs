use ozonide_core::estimation::PassthroughStateEstimator;

#[embassy_executor::task]
pub async fn state_estimation_task(
    state_estimator: &'static mut PassthroughStateEstimator,
) {
    ozonide_core::tasks::state_estimation_task(state_estimator).await;
}
