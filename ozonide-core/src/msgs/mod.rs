//! Wire-format message types shared between all binaries.
//!
//! Every type here is `Clone + Copy + serde::Serialize + serde::Deserialize`,
//! making them suitable for both in-process pub/sub and over-the-wire
//! serialisation (UDP, WebSocket) in the SITL pipeline.

mod actuator_command;
mod imu_data;
mod setpoints;
mod vehicle_state;

pub use actuator_command::ActuatorCommand;
pub use imu_data::ImuData;
pub use setpoints::{AngularVelocitySetpoint, AttitudeSetpoint, TorqueSetpoint};
pub use vehicle_state::VehicleState;
