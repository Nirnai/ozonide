//! Shared flight-controller logic for ozonide.
//!
//! This crate is `no_std` and contains everything that is reused between the
//! embedded `firmware` binary and the `sitl` Linux binary:
//!
//! - [`control`] - control algorithms and logic
//! - [`estimation`] - state estimation algorithms
//! - [`msgs`] — wire-format data types (e.g. [`msgs::ImuData`])
//! - [`topics`] — pub/sub primitives built on Embassy's `Watch`
//! - [`traits`] — hardware-agnostic driver interfaces
//! - [`tasks`] — async task bodies shared across binaries
#![no_std]

pub mod config;
pub mod control;
pub mod estimation;
pub mod filter;
pub mod msgs;
pub mod tasks;
pub mod topics;
pub mod traits;
