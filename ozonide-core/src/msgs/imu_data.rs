use serde::{Deserialize, Serialize};

/// A single IMU measurement in SI units, body frame.
///
/// Produced by [`crate::traits::ImuSource::read`] and published on
/// [`crate::topics::IMU_TOPIC`] at the configured ODR (default 1 kHz).
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct ImuData {
    /// Microseconds since boot, taken at the moment the DRDY interrupt fired.
    pub timestamp_us: u64,
    /// Linear acceleration in m/s², body frame (X forward, Y left, Z up).
    pub linear_acceleration: [f32; 3],
    /// Angular velocity in rad/s, body frame.
    pub angular_velocity: [f32; 3],
    /// Die temperature in °C, used for gyroscope bias compensation.
    pub temperature: f32,
}
