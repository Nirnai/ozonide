use nalgebra::{Vector3, Vector4};

use crate::filter::{Filter, FilterFamily, lowpass};
use crate::msgs::VehicleState;

/// One sample's conditioned outputs, mutually phase-aligned.
pub struct ConditionedSignals {
    /// Filtered angular velocity ω_f (rad/s²). Used for the rate-P term.
    pub angular_rate_filtered: Vector3<f32>,
    /// Filtered angular acceleration estimate α_f (rad/s²).
    pub angular_acceleration: Vector3<f32>,
    /// Modeled specific thrust in g's, from RPM: `thrust_coeff · u₀` (the thrust
    /// row of the effectiveness matrix applied to the filtered actuator state).
    /// A feed-forward of the achieved collective — NOT an accelerometer reading —
    /// so the rate loop tracks thrust but leaves vertical disturbance rejection
    /// to the outer loop (avoids double-counting). Hover ≈ 1.0.
    pub specific_thrust: f32,
    /// Filtered actuator state u₀_f (Ω², rad²/s²).
    pub actuator_state: Vector4<f32>,
    /// True if u₀ came from measured eRPM this sample; false means lag model.
    pub actuator_measured: bool,
}

/// First-order motor lag model: Ω tracks Ω_cmd with time constant `tau`.
///
/// Used as the u₀ source when eRPM telemetry is absent, and kept current
/// via `feed_command` so it can serve as a seamless fallback when telemetry
/// drops out mid-flight.
struct ActuatorLagModel {
    omega: Vector4<f32>,
    tau: f32,
}

impl ActuatorLagModel {
    fn new(tau: f32) -> Self {
        Self { omega: Vector4::zeros(), tau }
    }

    fn omega(&self) -> Vector4<f32> {
        self.omega
    }

    /// Euler step: dΩ/dt = (Ω_cmd − Ω) / tau.
    fn step_toward(&mut self, omega_cmd: &Vector4<f32>, dt: f32) {
        let alpha = dt / (self.tau + dt);
        self.omega += alpha * (omega_cmd - self.omega);
    }
}

/// INDI signal conditioning: produces the synchronized (α_f, thrust_f, u₀_f)
/// trio for the increment law. Runs on **gyro + RPM only** — the accelerometer
/// is not used here. The thrust channel is modeled from the actuator state, so
/// the rate loop only *tracks* collective; vertical disturbance rejection is the
/// outer loop's job, not this one.
///
/// # Synchronization invariant
///
/// The gyro and actuator paths pass through low-pass filters with **identical
/// coefficients**, so they share one group delay. The thrust channel is derived
/// from the already-filtered actuator state, so it inherits that delay for free.
/// The increment law subtracts these signals against each other; a delay mismatch
/// injects phase error into the loop's most sensitive arithmetic. This struct is
/// the single owner of that invariant — do not filter these signals anywhere
/// else, and do not tune one path's cutoff independently of the others.
///
/// # Angular acceleration estimation
///
/// Angular acceleration is estimated by differentiating the **already-filtered**
/// ω signal, then filtering α at the same cutoff. Differentiating the raw signal
/// would amplify high-frequency noise by 2πf; differentiating the filtered signal
/// first squashes that noise at a cost of group delay. Applying the same LPF to
/// α afterwards aligns ω and α in phase — both see exactly two filter delays —
/// which the allocation arithmetic requires.
///
/// `f_cut` is a control-design parameter (trades α noise for loop phase margin)
/// and must be tuned jointly with the rate-loop gains. Typical range: 10–30 Hz.
pub struct AngularRateConditioning {
    gyro_low_pass_filter: [Filter; 3],
    angular_rate_filtered_prev: Vector3<f32>,
    actuator_low_pass_filter: [Filter; 4],
    /// Thrust row of the effectiveness matrix (g's per Ω², per motor). Maps the
    /// filtered actuator state to the modeled specific thrust.
    thrust_coeff: Vector4<f32>,
    lag_model: ActuatorLagModel,
    dt: f32,
    primed: bool,
}

impl AngularRateConditioning {
    pub fn new(f_cut: f32, sample_rate: f32, lag_tau: f32, thrust_coeff: Vector4<f32>) -> Self {
        // Bessel, not Butterworth: flat group delay matters more here than
        // magnitude flatness — the delay sits directly in the rate loop, and
        // Bessel keeps it constant across the maneuver spectrum (see notes).
        let make_low_pass = || lowpass(sample_rate, f_cut, FilterFamily::Bessel, 2);
        Self {
            gyro_low_pass_filter: core::array::from_fn(|_| make_low_pass()),
            angular_rate_filtered_prev: Vector3::zeros(),
            actuator_low_pass_filter: core::array::from_fn(|_| make_low_pass()),
            thrust_coeff,
            lag_model: ActuatorLagModel::new(lag_tau),
            dt: 1.0 / sample_rate,
            primed: false,
        }
    }

    pub fn step(&mut self, state: &VehicleState) -> ConditionedSignals {
        // --- Gyro path: filter ONCE, then differentiate. ---
        // d/dt of the filtered rate IS the synchronized angular acceleration:
        // alpha_f = d/dt(H·angular_rate) = H·(true alpha), one filter delay,
        // matching the actuator and thrust paths. No second filter — adding
        // one would double this path's delay and desync it from u0.
        let angular_rate_filtered = Vector3::from_fn(|i, _| {
            self.gyro_low_pass_filter[i].process(state.angular_velocity[i])
        });

        let angular_acceleration = if self.primed {
            (angular_rate_filtered - self.angular_rate_filtered_prev) / self.dt
        } else {
            self.primed = true;
            Vector3::zeros()
        };
        self.angular_rate_filtered_prev = angular_rate_filtered;

        // --- Actuator path: filter Ω². ---
        let (omega_motors, actuator_measured) = match state.motor_speed() {
            Some(m) => (m, true),
            None => (self.lag_model.omega(), false),
        };
        let actuator_state = Vector4::from_fn(|i, _| {
            self.actuator_low_pass_filter[i].process(omega_motors[i] * omega_motors[i])
        });

        // --- Thrust path: modeled from the actuator state, NOT the accelerometer. ---
        // specific_thrust = (G·u₀)[3] = thrust_coeff · u₀, so the thrust channel is
        // a feed-forward tracker of the achieved collective. Vertical disturbance
        // rejection belongs to the outer loop — feeding the accelerometer here too
        // would double-count it. Inherits the actuator path's filter delay for free.
        let specific_thrust = self.thrust_coeff.dot(&actuator_state);

        ConditionedSignals {
            angular_rate_filtered,
            angular_acceleration,
            specific_thrust,
            actuator_state,
            actuator_measured,
        }
    }

    /// Feed the latest clamped Ω² command back into the lag model.
    ///
    /// Call this after every allocator step. The lag model stays synchronized
    /// with the commanded state so it can seamlessly take over if eRPM
    /// telemetry drops out.
    pub fn feed_command(&mut self, u_cmd_clamped: &Vector4<f32>) {
        let omega_cmd = u_cmd_clamped.map(|u| libm::sqrtf(u.max(0.0)));
        self.lag_model.step_toward(&omega_cmd, self.dt);
    }

    /// Warm-start all filters from the current vehicle state.
    ///
    /// Drives every filter to its DC steady state by replaying the current
    /// sensor values for enough samples that transients die out. Call on arm
    /// and after any controller reset. A 2nd-order Butterworth at ≥10 Hz /
    /// 1 kHz settles in fewer than 300 samples; 500 gives a comfortable margin.
    pub fn reset(&mut self, state: &VehicleState) {
        let av = &state.angular_velocity;
        let motors = state.motor_speed().unwrap_or_else(Vector4::zeros);

        for _ in 0..500 {
            for i in 0..3 {
                self.gyro_low_pass_filter[i].process(av[i]);
            }
            for i in 0..4 {
                self.actuator_low_pass_filter[i].process(motors[i] * motors[i]);
            }
        }

        self.angular_rate_filtered_prev = Vector3::new(av[0], av[1], av[2]);
        self.lag_model.omega = motors;
        self.primed = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::StateValidity;

    fn make_state(angular_velocity: [f32; 3], specific_force: [f32; 3]) -> VehicleState {
        VehicleState {
            angular_velocity,
            specific_force,
            ..VehicleState::default()
        }
    }

    fn make_state_with_motors(
        angular_velocity: [f32; 3],
        specific_force: [f32; 3],
        motor_speed: [f32; 4],
    ) -> VehicleState {
        VehicleState {
            angular_velocity,
            specific_force,
            motor_speed,
            valid: StateValidity::MOTOR_SPEED,
            ..VehicleState::default()
        }
    }

    const FS: f32 = 1000.0;
    const F_CUT: f32 = 20.0;
    const LAG_TAU: f32 = 0.05;
    const HOVER_OMEGA: f32 = 500.0;
    // 4 motors at HOVER_OMEGA model 1 g of specific thrust: 4·coeff·Ω² = 1.
    const THRUST_COEFF: f32 = 1.0 / (4.0 * HOVER_OMEGA * HOVER_OMEGA);

    fn make_conditioning() -> AngularRateConditioning {
        AngularRateConditioning::new(F_CUT, FS, LAG_TAU, Vector4::repeat(THRUST_COEFF))
    }

    /// After many steps with constant angular velocity, α must converge to zero.
    #[test]
    fn constant_rate_gives_zero_acceleration() {
        let mut cond = make_conditioning();
        let state = make_state([0.5, -0.3, 0.1], [0.0; 3]);

        let mut out = cond.step(&state);
        for _ in 0..2000 {
            out = cond.step(&state);
        }

        let alpha = out.angular_acceleration;
        assert!(alpha[0].abs() < 0.01, "roll α = {} (expected ≈ 0)", alpha[0]);
        assert!(alpha[1].abs() < 0.01, "pitch α = {} (expected ≈ 0)", alpha[1]);
        assert!(alpha[2].abs() < 0.01, "yaw α = {} (expected ≈ 0)", alpha[2]);
    }

    /// A step in roll rate must produce a measurable α transient.
    #[test]
    fn rate_step_produces_nonzero_acceleration() {
        let mut cond = make_conditioning();

        // Settle at zero rate.
        let zero = make_state([0.0; 3], [0.0; 3]);
        for _ in 0..500 {
            cond.step(&zero);
        }

        // Step to 1 rad/s roll and capture the transient peak.
        let step = make_state([1.0, 0.0, 0.0], [0.0; 3]);
        let mut peak_alpha = 0.0_f32;
        for _ in 0..200 {
            let out = cond.step(&step);
            peak_alpha = peak_alpha.max(out.angular_acceleration[0]);
        }

        assert!(
            peak_alpha > 0.5,
            "roll α peak = {} (expected > 0.5 rad/s² during 1 rad/s step)",
            peak_alpha
        );
    }

    /// Specific thrust is modeled from RPM (not the accelerometer): hover Ω² → 1 g.
    #[test]
    fn hover_rpm_maps_to_unity_thrust() {
        let mut cond = make_conditioning();
        let state = make_state_with_motors([0.0; 3], [0.0; 3], [HOVER_OMEGA; 4]);

        let mut out = cond.step(&state);
        for _ in 0..2000 {
            out = cond.step(&state);
        }

        assert!(
            (out.specific_thrust - 1.0).abs() < 0.01,
            "hover specific_thrust = {} (expected ≈ 1.0)",
            out.specific_thrust
        );
    }

    /// When MOTOR_SPEED is valid, actuator_measured must be true and u₀ ≈ Ω².
    #[test]
    fn actuator_state_uses_measured_rpm_when_available() {
        let mut cond = make_conditioning();
        let omega_m = 500.0_f32;
        let state = make_state_with_motors(
            [0.0; 3],
            [0.0; 3],
            [omega_m; 4],
        );

        let mut out = cond.step(&state);
        for _ in 0..2000 {
            out = cond.step(&state);
        }

        assert!(out.actuator_measured, "actuator_measured must be true when MOTOR_SPEED is set");
        let expected_u = omega_m * omega_m;
        for i in 0..4 {
            assert!(
                (out.actuator_state[i] - expected_u).abs() < expected_u * 0.01,
                "actuator_state[{}] = {} (expected ≈ {})",
                i, out.actuator_state[i], expected_u
            );
        }
    }

    /// When MOTOR_SPEED is not valid, actuator_measured must be false (lag model).
    #[test]
    fn actuator_falls_back_to_lag_model_when_no_rpm() {
        let mut cond = make_conditioning();
        let state = make_state([0.0; 3], [0.0; 3]);

        // Feed a command so the lag model has something to track.
        let u_cmd = Vector4::from_element(500.0_f32.powi(2));
        for _ in 0..500 {
            cond.step(&state);
            cond.feed_command(&u_cmd);
        }

        let out = cond.step(&state);
        assert!(
            !out.actuator_measured,
            "actuator_measured must be false when MOTOR_SPEED is not set"
        );
        for i in 0..4 {
            assert!(
                out.actuator_state[i] > 1000.0,
                "actuator_state[{}] = {} (expected > 0 after lag model warm-up)",
                i, out.actuator_state[i]
            );
        }
    }

    /// After reset, the first step must not spike: α should start close to zero.
    #[test]
    fn reset_prevents_startup_spike() {
        let mut cond = make_conditioning();
        let state = make_state([0.5, -0.3, 0.1], [0.0; 3]);

        cond.reset(&state);
        let out = cond.step(&state);

        for i in 0..3 {
            assert!(
                out.angular_acceleration[i].abs() < 1.0,
                "angular_acceleration[{}] = {} after reset (expected < 1.0 rad/s²)",
                i, out.angular_acceleration[i]
            );
        }
    }
}
