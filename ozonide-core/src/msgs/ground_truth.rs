use serde::{Deserialize, Serialize};

/// Ground-truth position and velocity from the simulator, or a navigation
/// system (GPS, VIO) in firmware. Published at IMU rate in SITL.
///
/// In SITL this is exact simulator state, no noise. In firmware it would be
/// replaced by a filtered navigation estimate.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct GroundTruthState {
    pub timestamp_us: u64,
    /// Position in world frame (ENU), metres.
    pub position: [f32; 3],
    /// Linear velocity in world frame (ENU), m/s.
    pub linear_velocity: [f32; 3],
    /// Attitude quaternion, stored `[x, y, z, w]` (scalar last). Active rotation
    /// body FLU → world ENU, same convention as [`crate::msgs::VehicleState::attitude`].
    pub attitude: [f32; 4],
}
