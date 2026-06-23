use nalgebra::Vector3;

use crate::msgs::{ActuatorCommand, VehicleState, VelocitySetpoint};
use crate::traits::Controller;

use super::attitude_controller::AttitudeController;
use super::angular_velocity_controller::AngularVelocityController;
use super::velocity_controller::VelocityController;

/// Three-loop INDI cascade: velocity P → attitude P → INDI rate loop.
///
/// ```text
/// VelocitySetpoint
///        │
///  VelocityController    (horizontal P, stateless)
///        │
///  AttitudeSetpoint
///        │
///  AttitudeController    (quaternion P, stateless)
///        │
///  AngularVelocitySetpoint
///        │
///  AngularVelocityController  (INDI rate loop, has state)
///        │
///  ActuatorCommand
/// ```
pub struct CascadedController {
    velocity: VelocityController,
    attitude: AttitudeController,
    rate: AngularVelocityController,
}

impl CascadedController {
    pub fn new(
        velocity: VelocityController,
        attitude_gain: Vector3<f32>,
        rate: AngularVelocityController,
    ) -> Self {
        Self { velocity, attitude: AttitudeController::new(attitude_gain), rate }
    }
}

impl Controller for CascadedController {
    type Setpoint = VelocitySetpoint;

    fn step(&mut self, state: &VehicleState, setpoint: &VelocitySetpoint) -> ActuatorCommand {
        let attitude_sp = self.velocity.compute(state, setpoint);
        let rate_sp     = self.attitude.compute(state, &attitude_sp);
        self.rate.step(state, &rate_sp)
    }

    fn reset(&mut self, state: &VehicleState) {
        // Velocity and attitude controllers are stateless — only rate needs warm-start.
        self.rate.reset(state);
    }
}
