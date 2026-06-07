use crate::msgs::VehicleState;
use crate::control::ControlDemand;

pub trait Controller {
    type Setpoint;
    fn update(&mut self, state: &VehicleState, setpoint: &Self::Setpoint) -> ControlDemand;
    fn reset(&mut self);
}

// pub trait VelocityController;
// pub trait PositionController;  