#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InverseActuatorModelError {
    /// c1 (slope) must be finite and positive.
    NonPositiveSlope,
    /// reference_voltage and nominal_voltage must be finite and positive.
    NonPositiveVoltage,
    /// idle_throttle must be in [0, 1).
    InvalidIdleThrottle,
}

#[derive(Debug, PartialEq)]
pub struct InverseActuatorModel {
    /// Intercept (rad/s).
    c0: f32,
    /// Slope (rad/s per unit throttle)
    c1: f32,
    /// Voltage at which the intercept and slope where determined
    reference_voltage: f32,
    /// Fallback Voltage
    nominal_voltage: f32,
    /// Idle floor
    idle_throttle: f32,
}

impl InverseActuatorModel {
    pub fn new(
        c0: f32,
        c1: f32,
        reference_voltage: f32,
        nominal_voltage: f32,
        idle_throttle: f32,
    ) -> Result<Self, InverseActuatorModelError> {
        if c1 <= 0.0 || !c1.is_finite() || !c0.is_finite() {
            return Err(InverseActuatorModelError::NonPositiveSlope);
        }
        if reference_voltage <= 0.0 || !reference_voltage.is_finite()
            || nominal_voltage <= 0.0 || !nominal_voltage.is_finite()
        {
            return Err(InverseActuatorModelError::NonPositiveVoltage);
        }
        if !(0.0..1.0).contains(&idle_throttle) {
            return Err(InverseActuatorModelError::InvalidIdleThrottle);
        }
        Ok(Self { c0, c1, reference_voltage, nominal_voltage, idle_throttle })
    }

    pub fn throttle(&self, omega_square: f32, battery_voltage: Option<f32>) -> f32 {
        let voltage = battery_voltage.unwrap_or(self.nominal_voltage);
        let omega = libm::sqrtf(omega_square.max(0.0));
        let throttle = (omega * self.reference_voltage / voltage - self.c0) / self.c1;
        throttle.clamp(self.idle_throttle, 1.0)
    }

    pub fn omega(&self, throttle: f32, battery_voltage: Option<f32>) -> f32 {
        let voltage = battery_voltage.unwrap_or(self.nominal_voltage);
        (voltage / self.reference_voltage) * (self.c0 + self.c1 * throttle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Baseline model used by most tests: identified at 11.1 V, 500–800 rad/s range.
    fn nominal() -> InverseActuatorModel {
        InverseActuatorModel::new(500.0, 3000.0, 11.1, 11.1, 0.05).unwrap()
    }

    // --- Constructor validation ---

    #[test]
    fn rejects_zero_slope() {
        assert_eq!(
            InverseActuatorModel::new(500.0, 0.0, 11.1, 11.1, 0.05),
            Err(InverseActuatorModelError::NonPositiveSlope)
        );
    }

    #[test]
    fn rejects_negative_slope() {
        assert_eq!(
            InverseActuatorModel::new(500.0, -1.0, 11.1, 11.1, 0.05),
            Err(InverseActuatorModelError::NonPositiveSlope)
        );
    }

    #[test]
    fn rejects_nan_slope() {
        assert_eq!(
            InverseActuatorModel::new(500.0, f32::NAN, 11.1, 11.1, 0.05),
            Err(InverseActuatorModelError::NonPositiveSlope)
        );
    }

    #[test]
    fn rejects_nan_intercept() {
        assert_eq!(
            InverseActuatorModel::new(f32::NAN, 3000.0, 11.1, 11.1, 0.05),
            Err(InverseActuatorModelError::NonPositiveSlope)
        );
    }

    #[test]
    fn rejects_zero_reference_voltage() {
        assert_eq!(
            InverseActuatorModel::new(500.0, 3000.0, 0.0, 11.1, 0.05),
            Err(InverseActuatorModelError::NonPositiveVoltage)
        );
    }

    #[test]
    fn rejects_negative_nominal_voltage() {
        assert_eq!(
            InverseActuatorModel::new(500.0, 3000.0, 11.1, -1.0, 0.05),
            Err(InverseActuatorModelError::NonPositiveVoltage)
        );
    }

    #[test]
    fn rejects_infinite_voltage() {
        assert_eq!(
            InverseActuatorModel::new(500.0, 3000.0, f32::INFINITY, 11.1, 0.05),
            Err(InverseActuatorModelError::NonPositiveVoltage)
        );
    }

    #[test]
    fn rejects_idle_throttle_at_one() {
        assert_eq!(
            InverseActuatorModel::new(500.0, 3000.0, 11.1, 11.1, 1.0),
            Err(InverseActuatorModelError::InvalidIdleThrottle)
        );
    }

    #[test]
    fn rejects_negative_idle_throttle() {
        assert_eq!(
            InverseActuatorModel::new(500.0, 3000.0, 11.1, 11.1, -0.1),
            Err(InverseActuatorModelError::InvalidIdleThrottle)
        );
    }

    #[test]
    fn accepts_zero_idle_throttle() {
        assert!(InverseActuatorModel::new(500.0, 3000.0, 11.1, 11.1, 0.0).is_ok());
    }

    // --- throttle() bounds ---

    #[test]
    fn throttle_clamps_negative_omega_square_to_idle() {
        // Negative Ω² is physically impossible; clamp to idle rather than panic.
        let m = nominal();
        assert_eq!(m.throttle(-1.0, None), m.idle_throttle);
    }

    #[test]
    fn throttle_clamps_very_large_omega_square_to_one() {
        let m = nominal();
        assert_eq!(m.throttle(f32::MAX, None), 1.0);
    }

    #[test]
    fn throttle_is_never_below_idle() {
        let m = nominal();
        // Ω² = 0 is below the model's linear range; output must still be ≥ idle_throttle.
        assert!(m.throttle(0.0, None) >= m.idle_throttle);
    }

    #[test]
    fn throttle_is_never_above_one() {
        let m = nominal();
        assert!(m.throttle(f32::MAX, None) <= 1.0);
    }

    // --- Round-trip consistency ---

    #[test]
    fn throttle_and_omega_are_inverses_at_reference_voltage() {
        let m = nominal();
        let throttle_in = 0.5_f32;
        let omega = m.omega(throttle_in, Some(11.1));
        let throttle_out = m.throttle(omega * omega, Some(11.1));
        assert!(
            (throttle_out - throttle_in).abs() < 1e-4,
            "round-trip error: in={throttle_in}, out={throttle_out}"
        );
    }

    #[test]
    fn throttle_and_omega_are_inverses_at_low_voltage() {
        let m = nominal();
        let throttle_in = 0.6_f32;
        let v = 9.6_f32;
        let omega = m.omega(throttle_in, Some(v));
        let throttle_out = m.throttle(omega * omega, Some(v));
        assert!(
            (throttle_out - throttle_in).abs() < 1e-4,
            "round-trip error at {v} V: in={throttle_in}, out={throttle_out}"
        );
    }

    #[test]
    fn nominal_voltage_used_when_battery_is_none() {
        let m = nominal();
        let t_none = m.throttle(500.0_f32.powi(2), None);
        let t_nominal = m.throttle(500.0_f32.powi(2), Some(11.1));
        assert_eq!(t_none, t_nominal);
    }

    // --- Voltage compensation direction ---

    #[test]
    fn lower_voltage_requires_higher_throttle_for_same_speed() {
        let m = nominal();
        let omega_sq = 700.0_f32.powi(2);
        let t_full = m.throttle(omega_sq, Some(11.1));
        let t_low = m.throttle(omega_sq, Some(9.0));
        assert!(
            t_low > t_full,
            "lower voltage should need more throttle: t_full={t_full}, t_low={t_low}"
        );
    }
}
