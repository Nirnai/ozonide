//! Wire-format message types shared between all binaries.
//!
//! Every type here is `Clone + Copy + serde::Serialize + serde::Deserialize`,
//! making them suitable for both in-process pub/sub and over-the-wire
//! serialisation (UDP, WebSocket) in the SITL pipeline.

mod actuator_command;
mod actuator_telemetry;
mod battery_status;
mod ground_truth;
mod imu_data;
mod setpoints;
mod vehicle_state;

pub use actuator_command::ActuatorCommand;
pub use actuator_telemetry::ActuatorTelemetry;
pub use battery_status::BatteryStatus;
pub use ground_truth::GroundTruthState;
pub use imu_data::ImuData;
pub use setpoints::{
    AngularAccelerationSetpoint, AngularRateSetpoint, AttitudeSetpoint, VelocitySetpoint,
};
pub use vehicle_state::{StateValidity, VehicleState, STANDARD_GRAVITY};
