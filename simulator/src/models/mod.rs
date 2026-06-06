//! Sensor and actuator models for hardware-in-the-loop simulation.
//!
//! - [`ActuatorModel`]   — brushless motor + propeller dynamics (thrust, drag torque, lag)
//! - [`ImuModel`]        — MEMS IMU with white noise, constant bias, and random walk
//! - [`DisturbanceModel`] — external wind and gust forces (Gaussian or Ornstein-Uhlenbeck)

mod imu_model;
mod actuator_model;
mod disturbance_model;

pub use imu_model::{ImuModel, ImuNoise};
pub use actuator_model::ActuatorModel;
pub use disturbance_model::{DisturbanceModel, DisturbanceType};
