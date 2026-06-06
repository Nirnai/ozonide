use crate::msgs::{VehicleState, AttitudeSetpoint, ActuatorCommand};


pub trait AttitudeController {
    fn update(&mut self, state: &VehicleState, setpoint: &AttitudeSetpoint) -> ActuatorCommand;
    fn reset(&mut self);
}

// pub trait VelocityController;
// pub trait PositionController;  