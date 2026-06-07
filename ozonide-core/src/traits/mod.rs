//! Hardware-agnostic driver traits.
//!
//! Each trait defines the contract a concrete driver must fulfil so that the
//! shared task bodies in [`crate::tasks`] can be reused across the embedded
//! firmware and the SITL binary without modification.

mod imu_source;
mod state_estimator;
mod controller;
mod actuator_sink;


pub use imu_source::ImuSource;
pub use actuator_sink::ActuatorSink;
pub use state_estimator::{StateEstimator, SensorData};
pub use controller::Controller;