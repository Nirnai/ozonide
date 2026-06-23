use nalgebra::Vector3;

use crate::msgs::{ActuatorCommand, VehicleState, VelocitySetpoint};
use crate::traits::Controller;

use super::attitude_controller::AttitudeController;
use super::angular_rate_controller::AngularRateController;
use super::velocity_controller::VelocityController;

/// Three-loop INDI cascade: outer INDI force loop → attitude P → INDI rate loop.
///
/// ```text
/// VelocitySetpoint
///        │
///  VelocityController     (outer INDI force loop, has state)
///        │
///  AttitudeSetpoint
///        │
///  AttitudeController     (quaternion P, stateless)
///        │
///  AngularRateSetpoint
///        │
///  AngularRateController  (INDI rate loop, has state)
///        │
///  ActuatorCommand
/// ```
pub struct CascadedController {
    velocity: VelocityController,
    attitude: AttitudeController,
    rate: AngularRateController,
}

impl CascadedController {
    pub fn new(
        velocity: VelocityController,
        attitude_gain: Vector3<f32>,
        rate: AngularRateController,
    ) -> Self {
        Self { velocity, attitude: AttitudeController::new(attitude_gain), rate }
    }
}

impl Controller for CascadedController {
    type Setpoint = VelocitySetpoint;

    fn step(&mut self, state: &VehicleState, setpoint: &VelocitySetpoint) -> ActuatorCommand {
        let attitude_sp = self.velocity.step(state, setpoint);
        let rate_sp     = self.attitude.compute(state, &attitude_sp);
        self.rate.step(state, &rate_sp)
    }

    fn reset(&mut self, state: &VehicleState) {
        // Attitude controller is stateless; the velocity and rate loops both
        // hold conditioning state and need warm-starting.
        self.velocity.reset(state);
        self.rate.reset(state);
    }
}
