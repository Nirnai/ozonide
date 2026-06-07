//! Async task bodies shared between the firmware and SITL binaries.
//!
//! Each function here is a plain `async fn` — not an
//! `#[embassy_executor::task]` — because the task macro requires a concrete
//! type and these functions are generic. Each binary provides a thin
//! one-line concrete wrapper that supplies the hardware-specific type and
//! delegates to the shared body here.

mod actuator_task;
mod control_task;
mod imu_task;
mod rate_monitor;
mod state_estimation_task;

pub use actuator_task::actuator_task;
pub use control_task::control_task;
pub use imu_task::imu_task;
pub use rate_monitor::rate_monitor;
pub use state_estimation_task::state_estimation_task;