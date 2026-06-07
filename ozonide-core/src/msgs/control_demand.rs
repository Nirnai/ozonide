use serde::{Deserialize, Serialize};

/// Virtual control axes produced by any [`crate::traits::Controller`] and
/// consumed by the control allocation stage.
///
/// All fields are normalised: `thrust` is in `[0, 1]`; torque values are in
/// `[-1, 1]` where the sign convention follows the body-frame right-hand rule.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct ControlDemand {
    /// Collective throttle demand. `0` = motors off, `1` = full power.
    pub thrust: f32,
    /// Roll torque demand. Positive = roll right.
    pub roll_torque: f32,
    /// Pitch torque demand. Positive = pitch forward.
    pub pitch_torque: f32,
    /// Yaw torque demand. Positive = rotate CCW from above.
    pub yaw_torque: f32,
}
