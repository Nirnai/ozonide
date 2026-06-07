mod allocation;
mod authority;
mod cascaded_pid_controller;

pub use allocation::allocate_control;
pub use cascaded_pid_controller::{AttitudeGains, AngularVelocityGains, CascadedPidController, PidGains};
pub use crate::msgs::ControlDemand;