use serde::{Deserialize, Serialize};

/// Motor throttle command produced by the control loop and consumed by the simulator.
///
/// Motor order (X config, top view, Z up): FR=0, RL=1, FL=2, RR=3.
/// Throttle is normalised to [0.0, 1.0].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize, Default)]
pub struct ActuatorCommand {
    /// Normalised throttle per motor in [0.0, 1.0].
    pub motor_throttle: [f32; 4],
}
