use ozonide_core::msgs::AngularVelocitySetpoint;
use ozonide_core::traits::SetpointSource;

/// Fixed setpoint source that always returns the same [`AngularVelocitySetpoint`].
///
/// Used in SITL to command constant angular rates. Replace with a topic
/// subscriber or RC parser when pilot input is wired up.
pub struct SetpointSimulated {
    setpoint: AngularVelocitySetpoint,
}

impl SetpointSimulated {
    pub fn new(setpoint: AngularVelocitySetpoint) -> Self {
        Self { setpoint }
    }
}

impl SetpointSource for SetpointSimulated {
    type Setpoint = AngularVelocitySetpoint;

    fn latest(&mut self) -> AngularVelocitySetpoint {
        self.setpoint
    }
}
