mod allocation;
mod authority;
mod cascaded_pid_controller;

pub use allocation::allocate_control;
pub use cascaded_pid_controller::CascadedPidController;


#[derive(Default)]
pub struct ControlDemand {
    pub thrust: f32,
    pub roll_torque: f32, 
    pub pitch_torque: f32,
    pub yaw_torque: f32
}