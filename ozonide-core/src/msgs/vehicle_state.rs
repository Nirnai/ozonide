use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize)]
/// Estimated vehicle state published by the state estimation task.
///
/// Fields that have no estimator yet default to zero. Consumers should only
/// read fields that are known to be populated for their use case.
pub struct VehicleState {
    /// Timestamp of the IMU sample that produced this estimate (µs since boot).
    pub timestamp_us: u64,
    /// Position in world frame, ENU (East-North-Up), metres.
    /// Defaults to `[0, 0, 0]` — no position estimator yet.
    pub position: [f32; 3],
    /// Attitude quaternion `[x, y, z, w]` (scalar last, matches nalgebra storage order).
    /// Represents the rotation from body frame to world frame (ENU).
    pub attitude: [f32; 4],
    /// Velocity in world frame, ENU, m/s.
    /// Defaults to `[0, 0, 0]` — no velocity estimator yet.
    pub linear_velocity: [f32; 3],
    /// Angular velocity in body frame, rad/s. Taken directly from the gyroscope.
    /// Order: `[roll_rate, pitch_rate, yaw_rate]`.
    pub angular_velocity: [f32; 3],
    /// Specific force in body frame, m/s². Taken directly from the accelerometer.
    /// Includes the effect of gravity — at rest with Z-up (ENU), reads approximately
    /// `[0, 0, 9.81]` because the accelerometer measures the surface reaction force,
    /// not gravitational acceleration.
    pub linear_acceleration: [f32; 3],
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            timestamp_us: 0,
            position: [0.0; 3],
            attitude: [0.0, 0.0, 0.0, 1.0], // identity quaternion [x,y,z,w]
            linear_velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
            linear_acceleration: [0.0; 3],
        }
    }
}
