use crate::ground_truth_simulated::GroundTruthSimulated;

#[embassy_executor::task]
pub async fn ground_truth_task(source: &'static mut GroundTruthSimulated) {
    source.run().await;
}
