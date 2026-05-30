use core::sync::atomic::{AtomicU32, Ordering};
use embassy_time::Timer;

/// Logs the publish rate of a topic once per second.
///
/// `name` is printed as a label; `count` is the topic's cumulative publish
/// counter — obtained via [`crate::topics::Topic::count`].
///
/// Wrap in a concrete `#[embassy_executor::task]` in each binary:
///
/// ```ignore
/// #[embassy_executor::task]
/// async fn imu_rate_monitor() {
///     ozonide_core::tasks::rate_monitor("IMU", IMU_TOPIC.count()).await;
/// }
/// ```
pub async fn rate_monitor(name: &'static str, count: &'static AtomicU32) {
    let mut prev = 0u32;
    loop {
        Timer::after_secs(1).await;
        let current = count.load(Ordering::Relaxed);
        #[cfg(feature = "defmt")]
        defmt::info!("{}: {} Hz", name, current - prev);
        prev = current;
    }
}
