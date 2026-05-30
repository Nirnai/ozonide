use crate::topics::IMU_TOPIC;
use crate::traits::ImuSource;

/// Core IMU acquisition loop, shared between firmware and SITL.
///
/// Initialises `source`, then loops forever: waits for a data-ready signal,
/// reads a sample, and publishes it to [`IMU_TOPIC`].
///
/// Each binary wraps this in a concrete `#[embassy_executor::task]` that
/// supplies the hardware-specific type:
///
/// ```ignore
/// #[embassy_executor::task]
/// async fn imu_task(imu: &'static mut Icm42688p<ImuSpi, ImuDrdy>) {
///     ozonide_core::tasks::imu_task(imu).await;
/// }
/// ```
pub async fn imu_task(source: &mut impl ImuSource) {
    source.init().await;
    #[cfg(feature = "defmt")]
    defmt::info!("IMU driver ready, entering sample loop");
    #[cfg(feature = "log")]
    log::info!("Simulated IMU ready, entering sample loop");
    let publisher = IMU_TOPIC.publisher();
    loop {
        source.data_ready().await;
        publisher.publish(source.read().await);
    }
}
