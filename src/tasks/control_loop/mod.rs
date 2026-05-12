//! Main control loop task
//!
//! Runs at high frequency (~500Hz) to compute motor outputs.

use crate::types::{VehicleState, ControlOutput};

/// Main control loop
pub struct ControlLoop {
    // TODO: Controller instance, setpoint, state
}

impl ControlLoop {
    /// Create new control loop
    pub fn new() -> Self {
        todo!("Initialize control loop")
    }

    /// Update control outputs based on current state
    /// Called at ~500Hz from main loop
    pub fn update(&mut self, state: &VehicleState) -> ControlOutput {
        todo!("Compute control output (MPC/PID)")
    }

    /// Set control mode (stabilize, altitude hold, position, etc.)
    pub fn set_mode(&mut self, mode: ControlMode) {
        todo!("Switch control mode")
    }

    /// Update reference/setpoint
    pub fn set_reference(&mut self, reference: &Reference) {
        todo!("Update desired state/trajectory")
    }

    // Future: Embassy task
    // #[cfg(feature = "embassy")]
    // pub async fn run(&mut self) {
    //     loop {
    //         let state = STATE_SIGNAL.wait().await;
    //         let output = self.update(&state);
    //         OUTPUT_CHANNEL.send(output).await;
    //         Timer::after_millis(2).await; // 500Hz
    //     }
    // }
}

#[derive(Debug, Clone, Copy)]
pub enum ControlMode {
    Disarmed,
    Manual,
    Stabilize,
    AltitudeHold,
    PositionHold,
    Autonomous,
}

/// Control reference/setpoint
#[derive(Debug, Clone)]
pub struct Reference {
    // TODO: Position, velocity, attitude setpoints
}
