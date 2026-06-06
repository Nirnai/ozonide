use crate::{
    control::cascaded_pid::{PidController, PidGains},
    msgs::VehicleState,
};

/// PID gains for all three angular rate axes.
///
/// Each axis has its own independent [`PidGains`] so roll, pitch, and yaw can be
/// tuned separately. Yaw typically uses lower gains than roll/pitch because the
/// yaw inertia is higher and the yaw actuator authority is weaker (reaction torque
/// from motor drag vs. direct thrust differential).
#[derive(Clone, Copy)]
pub struct RatePidGains {
    pub roll: PidGains,
    pub pitch: PidGains,
    pub yaw: PidGains,
}

/// Inner-loop angular rate controller.
///
/// Runs three independent PID controllers — one per body axis — that drive the
/// measured angular rates toward the commanded setpoints. This is the innermost
/// loop of a cascaded attitude controller:
///
/// ```text
/// AnglePID → rate setpoints → RatePidController → corrections → mixer → motors
///                                    ↑
///                               (this struct)
/// ```
///
/// ## Update law (per axis)
///
/// Given rate setpoint `ω_cmd` and measured rate `ω` from the gyroscope:
///
/// ```text
/// error = ω_cmd − ω
/// correction = Kp·error + Ki·∫error·dt + Kd·d(error)/dt
/// ```
///
/// The output `correction` is a normalised torque demand in `[-1, 1]` that is
/// passed directly to the control allocator.
///
/// ## Integrator state
///
/// Each axis holds its own integrator that accumulates error between calls.
/// Call [`reset`](RatePidController::reset) when switching flight modes or after
/// a long disarmed period to prevent stale integral wind-up from affecting the
/// first active update.
#[derive(Default)]
pub struct RatePidController {
    roll: PidController,
    pitch: PidController,
    yaw: PidController,
}

/// Normalised torque corrections produced by the rate PID for each body axis.
///
/// Values are in `[-1, 1]` and feed directly into `allocate_normalized_throttle_commands`.
/// Positive roll = roll right, positive pitch = pitch nose down, positive yaw = CCW from above.
pub struct RateCorrection {
    pub roll:  f32,
    pub pitch: f32,
    pub yaw:   f32,
}

impl RatePidController {
    /// Advances all three rate PID loops by one time step.
    ///
    /// # Arguments
    /// * `roll_rate_setpoint`  — desired roll rate (rad/s); from angle PID or pilot input
    /// * `pitch_rate_setpoint` — desired pitch rate (rad/s)
    /// * `yaw_rate_setpoint`   — desired yaw rate (rad/s); from pilot input directly
    /// * `estimate`            — current attitude estimate; only the `*_rate` fields are used
    /// * `gains`               — PID gains per axis (passed per-call for gain scheduling)
    /// * `dt`                  — time elapsed since previous call (s); returns zero correction if ≤ 0
    pub fn update(
        &mut self,
        roll_rate_setpoint: f32,
        pitch_rate_setpoint: f32,
        yaw_rate_setpoint: f32,
        state: &VehicleState,
        gains: &RatePidGains,
        dt: f32,
    ) -> RateCorrection {
        let roll_rate_error  = roll_rate_setpoint  - state.angular_velocity[0];
        let pitch_rate_error = pitch_rate_setpoint - state.angular_velocity[1];
        let yaw_rate_error   = yaw_rate_setpoint   - state.angular_velocity[2];

        let roll_correction = self.roll.update(roll_rate_error, &gains.roll, dt);
        let pitch_correction = self.pitch.update(pitch_rate_error, &gains.pitch, dt);
        let yaw_correction = self.yaw.update(yaw_rate_error, &gains.yaw, dt);
        RateCorrection { roll: roll_correction, pitch: pitch_correction, yaw: yaw_correction }
    }

    /// Resets all integrators and derivative state to zero.
    ///
    /// Call this on arming or mode transitions to prevent stale integral
    /// wind-up from affecting the first active control output.
    pub fn reset(&mut self) {
        self.roll = PidController::default();
        self.pitch = PidController::default();
        self.yaw = PidController::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-5;

    fn assert_approx(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < EPS, "got {actual}, expected {expected}");
    }

    fn p_gains(kp: f32) -> PidGains {
        PidGains { kp, ki: 0.0, kd: 0.0, integral_limit: 0.0, output_limit: f32::MAX }
    }

    fn zero_gains() -> PidGains {
        p_gains(0.0)
    }

    fn state_with_rates(roll_rate: f32, pitch_rate: f32, yaw_rate: f32) -> VehicleState {
        VehicleState { angular_velocity: [roll_rate, pitch_rate, yaw_rate], ..VehicleState::default() }
    }

    fn uniform_gains(kp: f32) -> RatePidGains {
        RatePidGains { roll: p_gains(kp), pitch: p_gains(kp), yaw: p_gains(kp) }
    }

    #[test]
    fn zero_error_produces_zero_correction() {
        let mut ctrl = RatePidController::default();
        let gains = uniform_gains(5.0);
        let estimate = state_with_rates(1.0, -0.5, 0.3);
        // Setpoints match measured rates → error = 0 → all corrections = 0.
        let out = ctrl.update(1.0, -0.5, 0.3, &estimate, &gains, 0.01);
        assert_approx(out.roll, 0.0);
        assert_approx(out.pitch, 0.0);
        assert_approx(out.yaw, 0.0);
    }

    #[test]
    fn proportional_correction_scales_with_error() {
        let mut ctrl = RatePidController::default();
        let gains = uniform_gains(2.0);
        // Measured rates all zero; setpoints give known errors.
        let estimate = state_with_rates(0.0, 0.0, 0.0);
        let out = ctrl.update(1.0, 0.5, -0.25, &estimate, &gains, 0.01);
        assert_approx(out.roll,  2.0 * 1.0);
        assert_approx(out.pitch, 2.0 * 0.5);
        assert_approx(out.yaw,   2.0 * -0.25);
    }

    #[test]
    fn axes_are_independent() {
        // Apply error on roll only; pitch and yaw corrections must stay zero.
        let mut ctrl = RatePidController::default();
        let gains = RatePidGains {
            roll:  p_gains(3.0),
            pitch: zero_gains(),
            yaw:   zero_gains(),
        };
        let estimate = state_with_rates(0.0, 0.0, 0.0);
        let out = ctrl.update(1.0, 0.0, 0.0, &estimate, &gains, 0.01);
        assert_approx(out.roll,  3.0);
        assert_approx(out.pitch, 0.0);
        assert_approx(out.yaw,   0.0);
    }

    #[test]
    fn zero_dt_returns_zero_correction() {
        let mut ctrl = RatePidController::default();
        let gains = uniform_gains(10.0);
        let estimate = state_with_rates(0.0, 0.0, 0.0);
        let out = ctrl.update(1.0, 1.0, 1.0, &estimate, &gains, 0.0);
        assert_approx(out.roll, 0.0);
        assert_approx(out.pitch, 0.0);
        assert_approx(out.yaw, 0.0);
    }

    #[test]
    fn reset_clears_integrator() {
        let mut ctrl = RatePidController::default();
        let gains = RatePidGains {
            roll:  PidGains { kp: 0.0, ki: 1.0, kd: 0.0, integral_limit: 10.0, output_limit: f32::MAX },
            pitch: PidGains { kp: 0.0, ki: 1.0, kd: 0.0, integral_limit: 10.0, output_limit: f32::MAX },
            yaw:   PidGains { kp: 0.0, ki: 1.0, kd: 0.0, integral_limit: 10.0, output_limit: f32::MAX },
        };
        let estimate = state_with_rates(0.0, 0.0, 0.0);
        // Wind up the integrators.
        for _ in 0..5 {
            ctrl.update(1.0, 1.0, 1.0, &estimate, &gains, 0.1);
        }
        ctrl.reset();
        // After reset the integral state is gone; only P and D terms remain (both 0 here).
        let out = ctrl.update(0.0, 0.0, 0.0, &estimate, &gains, 0.1);
        assert_approx(out.roll, 0.0);
        assert_approx(out.pitch, 0.0);
        assert_approx(out.yaw, 0.0);
    }
}
