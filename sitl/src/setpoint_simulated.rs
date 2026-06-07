use ozonide_core::msgs::AttitudeSetpoint;
use ozonide_core::traits::SetpointSource;

/// Fixed setpoint source that always returns the same [`AttitudeSetpoint`].
///
/// Used in SITL to command a constant hover attitude. Replace with a topic
/// subscriber or RC parser when pilot input is wired up.
pub struct SetpointSimulated {
    setpoint: AttitudeSetpoint,
}

impl SetpointSimulated {
    pub fn new(setpoint: AttitudeSetpoint) -> Self {
        Self { setpoint }
    }
}

impl SetpointSource for SetpointSimulated {
    type Setpoint = AttitudeSetpoint;

    async fn next(&mut self) -> AttitudeSetpoint {
        self.setpoint
    }
}
