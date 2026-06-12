//! Wire-format message types shared between all binaries.
//!
//! Every type here is `Clone + Copy + serde::Serialize + serde::Deserialize`,
//! making them suitable for both in-process pub/sub and over-the-wire
//! serialisation (UDP, WebSocket) in the SITL pipeline.

mod actuator_command;
mod battery_status;
mod imu_data;
mod motor_telemetry;
mod setpoints;
mod vehicle_state;

pub use actuator_command::ActuatorCommand;
pub use battery_status::BatteryStatus;
pub use imu_data::ImuData;
pub use motor_telemetry::MotorTelemetry;
pub use setpoints::{AttitudeSetpoint, AngularVelocitySetpoint, AngularAccelerationSetpoint};
pub use vehicle_state::{StateValidity, VehicleState, STANDARD_GRAVITY};
