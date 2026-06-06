/// Proportional, integral, and derivative gains for a single PID axis.
///
/// Gains are passed into [`PidController::update`] each call, keeping the
/// controller state decoupled from the gain values. This allows a gain
/// scheduler or parameter store to change gains at runtime without
/// resetting the integrator.
#[derive(Clone, Copy)]
pub struct PidGains {
    /// Proportional gain. Scales the output in proportion to the current error.
    /// Higher values give faster response but can cause oscillation.
    pub kp: f32,
    /// Integral gain. Accumulates error over time and eliminates steady-state offset.
    /// Too high causes integrator windup and slow, oscillatory corrections.
    pub ki: f32,
    /// Derivative gain. Damps the response by opposing the rate of change of error.
    /// Too high amplifies measurement noise.
    pub kd: f32,
    /// Symmetric clamp applied to the integrator after each update (output units).
    /// Limits the maximum contribution of the integral term to prevent windup
    /// when the actuator saturates.
    pub integral_limit: f32,
    /// Symmetric clamp applied to the total PID output (output units).
    /// Prevents the controller from commanding more than the actuator can deliver.
    /// Set to a large value (e.g. `f32::MAX`) to disable.
    pub output_limit: f32,
}

/// Single-axis discrete PID controller.
///
/// Holds only the state that changes between calls: the integrator accumulator
/// and the previous error for the derivative term. Gains are supplied per-call
/// via [`PidGains`].
///
/// ## Discrete update law
///
/// Given error `e[k]` and time step `dt`:
///
/// ```text
/// P = kp · e[k]
/// I = integrator_state + ki · e[k] · dt        (clamped for anti-windup)
/// D = kd · (e[k] − e[k−1]) / dt
///
/// output = P + I + D
/// ```
///
/// The integrator is clamped to `[-integral_limit, +integral_limit]` after
/// each update to prevent windup when the actuator saturates.
#[derive(Default)]
pub struct PidController {
    /// Accumulated integral term (output units).
    integrator_state: f32,
    /// Error from the previous call, used to compute the derivative term.
    previous_error: f32,
}

impl PidController {
    /// Advances the controller by one time step and returns the control output.
    ///
    /// # Arguments
    /// * `error` — setpoint minus measurement (positive = below target)
    /// * `gains` — proportional, integral, derivative gains and integral limit
    /// * `dt`    — time elapsed since the previous call (s)
    pub fn update(&mut self, error: f32, gains: &PidGains, dt: f32) -> f32 {
        if dt <= 0.0 {
            return 0.0;
        }
        let p = error * gains.kp;
        let i = (self.integrator_state + gains.ki * error * dt)
            .clamp(-gains.integral_limit, gains.integral_limit);
        let d = gains.kd * (error - self.previous_error) / dt;
        self.integrator_state = i;
        self.previous_error = error;
        (p + i + d).clamp(-gains.output_limit, gains.output_limit)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-5;

    fn assert_approx(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < EPS, "got {actual}, expected {expected}");
    }

    fn p_only(kp: f32) -> PidGains {
        PidGains { kp, ki: 0.0, kd: 0.0, integral_limit: 0.0, output_limit: f32::MAX }
    }

    fn i_only(ki: f32, integral_limit: f32) -> PidGains {
        PidGains { kp: 0.0, ki, kd: 0.0, integral_limit, output_limit: f32::MAX }
    }

    fn d_only(kd: f32) -> PidGains {
        PidGains { kp: 0.0, ki: 0.0, kd, integral_limit: 0.0, output_limit: f32::MAX }
    }

    #[test]
    fn zero_dt_returns_zero_and_does_not_update_state() {
        let mut pid = PidController::default();
        let gains = PidGains { kp: 1.0, ki: 1.0, kd: 1.0, integral_limit: 10.0, output_limit: f32::MAX };
        let output = pid.update(5.0, &gains, 0.0);
        assert_approx(output, 0.0);
        // Integrator must not have advanced.
        let next = pid.update(0.0, &gains, 0.1);
        assert_approx(next, 0.0); // error=0 → P=0, I still 0, D=(0−0)/dt=0
    }

    #[test]
    fn proportional_output_scales_with_error() {
        let mut pid = PidController::default();
        assert_approx(pid.update(3.0, &p_only(2.0), 0.1), 6.0);
        assert_approx(pid.update(-2.0, &p_only(2.0), 0.1), -4.0);
    }

    #[test]
    fn integral_accumulates_over_time() {
        // ki=1, error=1, dt=0.1 → each step adds 0.1 to integrator.
        let mut pid = PidController::default();
        let gains = i_only(1.0, 10.0);
        assert_approx(pid.update(1.0, &gains, 0.1), 0.1);
        assert_approx(pid.update(1.0, &gains, 0.1), 0.2);
        assert_approx(pid.update(1.0, &gains, 0.1), 0.3);
    }

    #[test]
    fn integral_is_clamped_by_limit() {
        // integral_limit=0.5: integrator must never exceed ±0.5 regardless of steps.
        let mut pid = PidController::default();
        let gains = i_only(1.0, 0.5);
        for _ in 0..20 {
            pid.update(1.0, &gains, 0.1);
        }
        assert_approx(pid.update(1.0, &gains, 0.1), 0.5);
    }

    #[test]
    fn derivative_responds_to_rate_of_change() {
        // First call: previous_error=0, error=1, dt=0.1 → D = kd*(1−0)/0.1 = 10.
        // Second call: error unchanged → D = kd*(1−1)/0.1 = 0.
        let mut pid = PidController::default();
        let gains = d_only(1.0);
        assert_approx(pid.update(1.0, &gains, 0.1), 10.0);
        assert_approx(pid.update(1.0, &gains, 0.1), 0.0);
    }

    #[test]
    fn output_is_clamped_by_output_limit() {
        // kp=10, error=1 → P=10, but output_limit=5 → output must be clamped to 5.
        let mut pid = PidController::default();
        let gains = PidGains { kp: 10.0, ki: 0.0, kd: 0.0, integral_limit: 0.0, output_limit: 5.0 };
        assert_approx(pid.update(1.0, &gains, 0.1), 5.0);
        assert_approx(pid.update(-1.0, &gains, 0.1), -5.0);
    }

    #[test]
    fn zero_error_produces_zero_output() {
        let mut pid = PidController::default();
        let gains = PidGains { kp: 5.0, ki: 2.0, kd: 1.0, integral_limit: 10.0, output_limit: f32::MAX };
        assert_approx(pid.update(0.0, &gains, 0.1), 0.0);
    }
}
