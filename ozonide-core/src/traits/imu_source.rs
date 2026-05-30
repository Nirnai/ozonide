//! Hardware-agnostic IMU driver interface.

use crate::msgs::ImuData;

/// Abstraction over any IMU backend — real hardware or a simulated source.
///
/// Implement this trait to plug a new IMU into [`crate::tasks::imu_task`]
/// without modifying any shared logic.
///
/// # Method contract
///
/// The methods must be called in order:
/// 1. [`init`](ImuSource::init) — configure the device once at startup.
/// 2. [`data_ready`](ImuSource::data_ready) — wait for a new sample to be
///    available (interrupt-driven or polled).
/// 3. [`read`](ImuSource::read) — burst-read and return the sample.
///
/// Steps 2–3 repeat in a tight loop for the lifetime of the task.
#[allow(async_fn_in_trait)]
pub trait ImuSource {
    /// Initialise the IMU. Called once before the sample loop begins.
    async fn init(&mut self);
    /// Wait until a new sample is ready (e.g. DRDY rising edge).
    async fn data_ready(&mut self);
    /// Read and return the latest sample in SI units, body frame.
    async fn read(&mut self) -> ImuData;
}
