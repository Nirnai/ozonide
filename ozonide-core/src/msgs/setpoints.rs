use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct TorqueSetpoint {
    /// Normalised roll torque demand `[-1, 1]`. Positive = roll right.
    pub roll_torque: f32,
    /// Normalised pitch torque demand `[-1, 1]`. Positive = pitch nose down.
    pub pitch_torque: f32,
    /// Normalised yaw torque demand `[-1, 1]`. Positive = rotate CCW from above.
    pub yaw_torque: f32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct AngularVelocitySetpoint {
    /// Desired roll rate (rad/s).
    pub roll_rate: f32,
    /// Desired pitch rate (rad/s).
    pub pitch_rate: f32,
    /// Desired yaw rate (rad/s). Passed through from [`AttitudeSetpoint::yaw_rate`] unchanged.
    pub yaw_rate: f32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct AttitudeSetpoint {
    /// Desired roll angle (rad). Positive = right side down (ENU, right-hand rule).
    pub roll: f32,
    /// Desired pitch angle (rad). Positive = nose down.
    pub pitch: f32,
    /// Desired yaw rate (rad/s). Positive = rotate counter-clockwise viewed from above.
    pub yaw_rate: f32,
    /// Normalised collective thrust [0.0, 1.0]. 0.0 = no thrust, 1.0 = all motors at max.
    pub thrust: f32,
}
