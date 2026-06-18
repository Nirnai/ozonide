//! Hardware-agnostic driver traits.
//!
//! Each trait defines the contract a concrete driver must fulfil so that the
//! shared task bodies in [`crate::tasks`] can be reused across the embedded
//! firmware and the SITL binary without modification.

mod actuator_sink;
mod actuator_telemetry_source;
mod controller;
mod imu_source;
mod setpoint_source;
mod state_estimator;

pub use actuator_sink::ActuatorSink;
pub use actuator_telemetry_source::ActuatorTelemetrySource;
pub use controller::Controller;
pub use imu_source::ImuSource;
pub use setpoint_source::SetpointSource;
pub use state_estimator::{SensorData, StateEstimator};