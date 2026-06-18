use nalgebra::Vector3;

use crate::msgs::{ActuatorCommand, AttitudeSetpoint, VehicleState};
use crate::traits::Controller;

use super::attitude_controller::AttitudeController;
use super::angular_velocity_controller::AngularVelocityController;

/// Two-loop INDI cascade: attitude P-controller → INDI angular-velocity controller.
///
/// ```text
/// AttitudeSetpoint
///        │
///  AttitudeController  (quaternion P, stateless)
///        │
///  AngularVelocitySetpoint
///        │
///  AngularVelocityController  (INDI rate loop)
///        │
///  ActuatorCommand
/// ```
pub struct CascadedController {
    attitude: AttitudeController,
    rate: AngularVelocityController,
}

impl CascadedController {
    /// `attitude_gain` — outer-loop P gain per axis `[roll, pitch, yaw]`, units: s⁻¹.
    /// Typical starting value: 3–6 s⁻¹ (lower than the rate-loop bandwidth).
    pub fn new(attitude_gain: Vector3<f32>, rate: AngularVelocityController) -> Self {
        Self { attitude: AttitudeController::new(attitude_gain), rate }
    }
}

impl Controller for CascadedController {
    type Setpoint = AttitudeSetpoint;

    fn step(&mut self, state: &VehicleState, setpoint: &AttitudeSetpoint) -> ActuatorCommand {
        let rate_setpoint = self.attitude.compute(state, setpoint);
        self.rate.step(state, &rate_setpoint)
    }

    fn reset(&mut self, state: &VehicleState) {
        // AttitudeController is stateless — only the rate controller needs warm-starting.
        self.rate.reset(state);
    }
}
