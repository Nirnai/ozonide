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
    /// Desired yaw rate (rad/s). Passed through from [`AttitudeSetpoint::yaw_rate`] unchanged.
    pub yaw_rate: f32,
    /// Collective specific force in g's, Hover ≈ 1.0. Max ≈ 3.0 - 5.0
    pub specific_thrust: f32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct AttitudeSetpoint {
    pub timestamp_us: u64,
    /// Desired roll angle (rad). Positive = right side down (ENU, right-hand rule).
    pub roll: f32,
    /// Desired pitch angle (rad). Positive = nose down.
    pub pitch: f32,
    /// Desired yaw rate (rad/s). Positive = rotate counter-clockwise viewed from above.
    pub yaw_rate: f32,
    /// Collective specific force in g's, Hover ≈ 1.0. Max ≈ 3.0 - 5.0
    pub specific_thrust: f32,
}
