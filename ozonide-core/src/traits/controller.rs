use crate::msgs::{ControlDemand, VehicleState};

pub trait Controller {
    type Setpoint;
    fn update(&mut self, state: &VehicleState, setpoint: &Self::Setpoint) -> ControlDemand;
    fn reset(&mut self);
}

// pub trait VelocityController;
// pub trait PositionController;  