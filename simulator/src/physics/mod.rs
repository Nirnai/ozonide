//! Rigid body dynamics for a quadrotor in the ENU world frame.
//!
//! The public entry point is [`step`], which advances the full vehicle state
//! by one time step using a 4th-order Runge-Kutta scheme. Supporting types
//! live in the sub-modules:
//!
//! - [`VehicleState`] — position, attitude, velocity, and rotor speeds
//! - [`VehicleParameters`] — airframe physical constants
//! - [`GRAVITATIONAL_CONSTANT`] — standard gravity (ISO 80000-3)

mod parameters;
mod state;
mod physics;

pub use parameters::{GRAVITATIONAL_CONSTANT, VehicleParameters};
pub use state::VehicleState;
pub use physics::step;
