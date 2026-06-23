use nalgebra::{Vector3, Vector4};

use crate::filter::{lowpass, Filter, FilterFamily};
use crate::msgs::{VehicleState, STANDARD_GRAVITY};

/// One sample's conditioned outer-loop signals, phase-aligned.
pub struct ConditionedForce {
    /// Measured specific force `f̂` in world frame (ENU), m/s². This is the
    /// *total* non-gravitational acceleration — thrust **plus** any specific-force
    /// disturbance (wind, drag). The INDI law's `ν̂`.
    pub specific_force: Vector3<f32>,
    /// Modeled thrust specific force `u₀` in world frame (ENU), m/s²: the thrust
    /// the motors are actually producing, from RPM (`(k_t/m)·ΣΩ²` along body-z).
    /// Excludes the disturbance — that is the whole point. The INDI law's `u₀`.
    pub thrust: Vector3<f32>,
}

/// Outer-loop signal conditioning: the specific-force analogue of
/// [`AngularRateConditioning`](super::angular_rate_conditioning::AngularRateConditioning),
/// producing the `(ν̂, u₀)` pair the outer INDI law differences.
///
/// The disturbance rejection hinges on `ν̂` and `u₀` coming from **different
/// sources**:
/// - `specific_force` (`ν̂`) — accelerometer, **includes** the disturbance;
/// - `thrust` (`u₀`) — RPM-derived thrust model, **excludes** the disturbance.
///
/// Their gap `f̂ − u₀` is the specific-force disturbance estimate the INDI
/// increment cancels. Both paths share identical filters (Bessel, flat group
/// delay) so they are phase-synchronized before the subtraction.
///
/// When motor telemetry is absent, `thrust` falls back to `specific_force`, which
/// degrades the outer loop to a stable proportional controller with no
/// disturbance rejection (you cannot separate thrust from disturbance without a
/// thrust model). The firmware fallback is a collective lag model — TODO, mirror
/// the rate loop's `ActuatorLagModel`.
pub struct SpecificForceConditioning {
    accel_low_pass: [Filter; 3],
    thrust_low_pass: Filter,
    /// Per-motor specific-thrust coefficient (g's per Ω², i.e. `k_t/(m·g)`):
    /// the thrust row of the control effectiveness matrix. `coeff · Ω² → g's`.
    thrust_coeff: Vector4<f32>,
}

impl SpecificForceConditioning {
    pub fn new(f_cut: f32, sample_rate: f32, thrust_coeff: Vector4<f32>) -> Self {
        let make_low_pass = || lowpass(sample_rate, f_cut, FilterFamily::Bessel, 2);
        Self {
            accel_low_pass: core::array::from_fn(|_| make_low_pass()),
            thrust_low_pass: make_low_pass(),
            thrust_coeff,
        }
    }

    /// Thrust specific force magnitude (m/s²) from motor RPM, or `None` if
    /// telemetry is absent. `|u₀| = g · (coeff · Ω²) = (k_t/m)·ΣΩ²`.
    fn thrust_magnitude(&self, state: &VehicleState) -> Option<f32> {
        state
            .motor_speed()
            .map(|omega| STANDARD_GRAVITY * self.thrust_coeff.dot(&omega.component_mul(&omega)))
    }

    pub fn step(&mut self, state: &VehicleState) -> ConditionedForce {
        let rotation = state.attitude(); // body FLU → world ENU

        let f_body = Vector3::from_fn(|i, _| self.accel_low_pass[i].process(state.specific_force[i]));
        let specific_force = rotation * f_body;

        let thrust = match self.thrust_magnitude(state) {
            Some(magnitude) => {
                let filtered = self.thrust_low_pass.process(magnitude);
                rotation * Vector3::new(0.0, 0.0, filtered) // thrust acts along body +z (FLU)
            }
            // No thrust telemetry → can't separate thrust from disturbance.
            None => specific_force,
        };

        ConditionedForce { specific_force, thrust }
    }

    /// Warm-start both filter paths to the current state, mirroring the rate
    /// path's reset so the outer loop has no cold-start transient.
    pub fn reset(&mut self, state: &VehicleState) {
        let f = &state.specific_force;
        let thrust_dc = self.thrust_magnitude(state).unwrap_or(STANDARD_GRAVITY);
        for _ in 0..500 {
            for i in 0..3 {
                self.accel_low_pass[i].process(f[i]);
            }
            self.thrust_low_pass.process(thrust_dc);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::StateValidity;
    use nalgebra::UnitQuaternion;

    const FS: f32 = 1000.0;
    const F_CUT: f32 = 10.0;
    const HOVER_OMEGA: f32 = 500.0;
    // 4 motors at HOVER_OMEGA must give 1 g: 4·coeff·HOVER_OMEGA² = 1.
    const THRUST_COEFF: f32 = 1.0 / (4.0 * HOVER_OMEGA * HOVER_OMEGA);

    fn cond() -> SpecificForceConditioning {
        SpecificForceConditioning::new(F_CUT, FS, Vector4::repeat(THRUST_COEFF))
    }

    fn hover_state() -> VehicleState {
        VehicleState {
            specific_force: [0.0, 0.0, STANDARD_GRAVITY],
            attitude: [0.0, 0.0, 0.0, 1.0],
            valid: StateValidity::ATTITUDE,
            ..VehicleState::default()
        }
    }

    fn hover_state_with_motors() -> VehicleState {
        VehicleState {
            motor_speed: [HOVER_OMEGA; 4],
            valid: StateValidity::ATTITUDE.with(StateValidity::MOTOR_SPEED),
            ..hover_state()
        }
    }

    #[test]
    fn level_hover_settles_to_world_up() {
        let mut c = cond();
        let state = hover_state();
        c.reset(&state);
        let out = c.step(&state);
        assert!(out.specific_force[0].abs() < 1e-3 && out.specific_force[1].abs() < 1e-3);
        assert!((out.specific_force[2] - STANDARD_GRAVITY).abs() < 1e-2);
    }

    #[test]
    fn body_force_is_rotated_to_world() {
        // 90° yaw: body-x specific force should appear along world-y (north).
        let yaw = UnitQuaternion::from_euler_angles(0.0, 0.0, core::f32::consts::FRAC_PI_2);
        let q = yaw.coords;
        let mut state = hover_state();
        state.attitude = [q.x, q.y, q.z, q.w];
        state.specific_force = [1.0, 0.0, STANDARD_GRAVITY];

        let mut c = cond();
        c.reset(&state);
        let out = c.step(&state);
        assert!(out.specific_force[1] > 0.5, "expected world-y > 0, got {:?}", out.specific_force);
        assert!(out.specific_force[0].abs() < 0.1);
    }

    #[test]
    fn thrust_from_rpm_recovers_hover_specific_force() {
        let mut c = cond();
        let state = hover_state_with_motors();
        c.reset(&state);
        let out = c.step(&state);
        // 1 g of thrust, pointing world-up at level attitude.
        assert!(out.thrust[0].abs() < 1e-2 && out.thrust[1].abs() < 1e-2);
        assert!((out.thrust[2] - STANDARD_GRAVITY).abs() < 1e-2, "thrust_z={}", out.thrust[2]);
    }

    #[test]
    fn thrust_excludes_disturbance_while_specific_force_includes_it() {
        // Eastward wind: accelerometer sees it, the RPM thrust model does not.
        let mut state = hover_state_with_motors();
        state.specific_force = [2.0, 0.0, STANDARD_GRAVITY]; // +x disturbance in f̂
        let mut c = cond();
        c.reset(&state);
        let out = c.step(&state);
        assert!((out.specific_force[0] - 2.0).abs() < 0.1, "f̂ must include wind: {:?}", out.specific_force);
        assert!(out.thrust[0].abs() < 1e-2, "u₀ must exclude wind: {:?}", out.thrust);
        // The gap f̂ − u₀ is the disturbance estimate.
        let d_hat = out.specific_force - out.thrust;
        assert!((d_hat[0] - 2.0).abs() < 0.1, "disturbance estimate wrong: {d_hat:?}");
    }

    #[test]
    fn falls_back_to_measured_without_telemetry() {
        let mut c = cond();
        let state = hover_state(); // no MOTOR_SPEED
        c.reset(&state);
        let out = c.step(&state);
        assert!((out.thrust - out.specific_force).norm() < 1e-6, "fallback: u₀ should equal f̂");
    }
}
