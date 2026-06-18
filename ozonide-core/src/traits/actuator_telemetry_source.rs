use crate::msgs::ActuatorTelemetry;

/// Abstraction over any ESC telemetry backend — bidirectional DShot/CAN in firmware
/// or a simulated source in SITL.
///
/// Implement this trait to plug a new backend into
/// [`crate::tasks::actuator_telemetry_task`] without modifying shared logic.
///
/// # Method contract
///
/// Methods must be called in order:
/// 1. [`init`](ActuatorTelemetrySource::init) — configure the peripheral once at startup.
/// 2. [`data_ready`](ActuatorTelemetrySource::data_ready) — wait for a new telemetry
///    frame (interrupt or poll).
/// 3. [`read`](ActuatorTelemetrySource::read) — return the frame.
///
/// Steps 2–3 repeat for the lifetime of the task.
#[allow(async_fn_in_trait)]
pub trait ActuatorTelemetrySource {
    async fn init(&mut self);
    async fn data_ready(&mut self);
    async fn read(&mut self) -> ActuatorTelemetry;
}
