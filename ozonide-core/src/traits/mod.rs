//! Hardware-agnostic driver traits.
//!
//! Each trait defines the contract a concrete driver must fulfil so that the
//! shared task bodies in [`crate::tasks`] can be reused across the embedded
//! firmware and the SITL binary without modification.

mod imu_source;
pub use imu_source::ImuSource;
