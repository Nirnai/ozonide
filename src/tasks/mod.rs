//! Task modules representing logical units of work
//!
//! Each task module is designed to eventually become an RTOS/Embassy task.
//! For now, they provide update() methods called from the main loop.

pub mod control_loop;
pub mod sensor_fusion;
// pub mod telemetry;
// pub mod vision;
