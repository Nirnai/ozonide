/// Abstraction over any setpoint provider — RC receiver, autopilot, or a test fixture.
///
/// Implement this trait to plug a new setpoint source into
/// [`crate::tasks::control_task`] without modifying any shared logic.
#[allow(async_fn_in_trait)]
pub trait SetpointSource {
    type Setpoint;
    /// Wait for and return the next setpoint. Blocks until one is available.
    async fn next(&mut self) -> Self::Setpoint;
}
