use crate::topics::ACTUATOR_TELEMETRY_TOPIC;
use crate::traits::ActuatorTelemetrySource;

/// Core ESC telemetry acquisition loop, shared between firmware and SITL.
///
/// Initialises `source`, then loops forever: waits for a telemetry frame,
/// reads it, and publishes it to [`ACTUATOR_TELEMETRY_TOPIC`].
///
/// Each binary wraps this in a concrete `#[embassy_executor::task]`:
///
/// ```ignore
/// #[embassy_executor::task]
/// async fn actuator_telemetry_task(src: &'static mut MyDshotDriver) {
///     ozonide_core::tasks::actuator_telemetry_task(src).await;
/// }
/// ```
pub async fn actuator_telemetry_task(source: &mut impl ActuatorTelemetrySource) {
    source.init().await;
    #[cfg(feature = "log")]
    log::info!("ActuatorTelemetrySource ready, entering telemetry loop");
    let publisher = ACTUATOR_TELEMETRY_TOPIC.publisher();
    loop {
        source.data_ready().await;
        publisher.publish(source.read().await);
    }
}
