use crate::msgs::{ActuatorCommand, VehicleState};

pub trait Controller {
    type Setpoint;
    /// Advance the controller by one sample period and produce the command
    /// for it. Must be called exactly once per `VehicleState` sample, in
    /// order — the controller's internal filters assume a fixed sample rate.
    fn step(&mut self, state: &VehicleState, setpoint: &Self::Setpoint) -> ActuatorCommand;
    fn reset(&mut self, state: &VehicleState);
}
