//! Wire-format message types shared between all binaries.
//!
//! Every type here is `Clone + Copy + serde::Serialize + serde::Deserialize`,
//! making them suitable for both in-process pub/sub and over-the-wire
//! serialisation (UDP, WebSocket) in the SITL pipeline.

mod imu_data;
mod attitude_setpoint;
mod actuator_command;


pub use imu_data::ImuData;
pub use attitude_setpoint::AttitudeSetpoint;
pub use actuator_command::ActuatorCommand;