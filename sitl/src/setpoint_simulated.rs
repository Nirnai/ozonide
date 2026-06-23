use ozonide_core::msgs::VelocitySetpoint;
use ozonide_core::traits::SetpointSource;

/// Fixed setpoint source that always returns the same [`VelocitySetpoint`].
///
/// Used in SITL to command a constant velocity (zero = hover in place).
pub struct SetpointSimulated {
    setpoint: VelocitySetpoint,
}

impl SetpointSimulated {
    pub fn new(setpoint: VelocitySetpoint) -> Self {
        Self { setpoint }
    }
}

impl SetpointSource for SetpointSimulated {
    type Setpoint = VelocitySetpoint;

    fn latest(&mut self) -> VelocitySetpoint {
        self.setpoint
    }
}
