use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct AngularAccelerationSetpoint {
    pub timestamp_us: u64,
    /// Desired roll acceleration (rad/s²). Positive = roll right.
    pub roll_acceleration: f32,
    /// Desired pitch acceleration (rad/s²). Positive = pitch nose down.
    pub pitch_acceleration: f32,
    /// Desired yaw acceleration (rad/s²). Positive = rotate CCW from above.
    pub yaw_acceleration: f32,
    /// Collective specific force in g's, Hover ≈ 1.0. Max ≈ 3.0 - 5.0
    pub specific_thrust: f32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct AngularVelocitySetpoint {
    pub timestamp_us: u64,
    /// Desired roll rate (rad/s).
    pub roll_rate: f32,
    /// Desired pitch rate (rad/s).
    pub pitch_rate: f32,
    /// Desired yaw rate (rad/s).
    pub yaw_rate: f32,
    /// Collective specific force in g's, Hover ≈ 1.0. Max ≈ 3.0 - 5.0
    pub specific_thrust: f32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct AttitudeSetpoint {
    pub timestamp_us: u64,
    /// Desired attitude quaternion, stored `[x, y, z, w]` (scalar last, nalgebra
    /// storage order). Same convention as [`VehicleState::attitude`].
    /// Active rotation: body FLU → world ENU.
    pub attitude: [f32; 4],
    /// Yaw rate feed-through (rad/s), superimposed on the attitude-error yaw
    /// output. Set by the velocity controller to pass pilot yaw commands through
    /// the cascade without needing an absolute yaw reference.
    pub yaw_rate: f32,
    /// Collective specific force in g's. Hover ≈ 1.0.
    pub specific_thrust: f32,
}

impl Default for AttitudeSetpoint {
    fn default() -> Self {
        Self { timestamp_us: 0, attitude: [0.0, 0.0, 0.0, 1.0], yaw_rate: 0.0, specific_thrust: 1.0 }
    }
}

/// Desired body velocity in the world (ENU) frame plus yaw rate.
///
/// Consumed by the [`VelocityController`](crate::control::indi::VelocityController),
/// which maps velocity error to a tilt [`AttitudeSetpoint`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct VelocitySetpoint {
    pub timestamp_us: u64,
    /// Desired velocity in world frame (ENU), m/s. `[East, North, Up]`.
    pub linear_velocity: [f32; 3],
    /// Desired yaw rate (rad/s). Passed through to the rate loop unchanged.
    pub yaw_rate: f32,
}
