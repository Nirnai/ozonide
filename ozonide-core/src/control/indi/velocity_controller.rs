use nalgebra::Vector3;

use crate::msgs::{AttitudeSetpoint, VehicleState, VelocitySetpoint, STANDARD_GRAVITY};

use super::incremental_inversion::IncrementalInversion;
use super::specific_force_conditioning::SpecificForceConditioning;
use super::thrust_vector_decomposition::ThrustVectorDecomposition;

/// Outer INDI loop: velocity error → desired specific force → INDI increment →
/// thrust vector → [`AttitudeSetpoint`]. Structurally the mirror of
/// [`AngularRateController`](super::angular_rate_controller::AngularRateController),
/// one cascade level up.
///
/// ```text
/// velocity-P:        a_des = K · (v_des − v̂)            (world ENU)
/// gravity feedfwd:   f_des = a_des + [0, 0, g]          (desired specific force, m/s²)
/// INDI increment:    τ = u₀ + B⁻¹·(f_des − f̂)           (B = I)
/// output map:        τ → AttitudeSetpoint               (ThrustVectorDecomposition)
/// ```
///
/// # Disturbance rejection
///
/// The two arguments to the increment come from *different sources*, which is
/// what makes this real INDI rather than a direct command:
/// - `f̂` (ν̂) — measured specific force (accelerometer), **includes** the
///   disturbance (wind, drag);
/// - `u₀` — thrust the motors actually produce, modeled from RPM, **excludes** it.
///
/// So `τ = u₀ + (f_des − f̂) = f_des − (f̂ − u₀) = f_des − d̂`, where `d̂ = f̂ − u₀`
/// is the specific-force disturbance estimate. INDI here is a disturbance
/// observer; it rejects what the accelerometer sees that the motors are not
/// producing. `u₀` is re-anchored to the actual thrust every cycle (not
/// accumulated), so there is no integrator and no windup.
///
/// Working in specific-force units keeps `B = I` (mass lives in the RPM thrust
/// model for `u₀`, not in `B`). The INDI clamp runs with infinite limits; tilt
/// and thrust saturation live in the decomposition.
///
/// Stateful only through the conditioning filters — needs [`reset`](Self::reset)
/// on arm to settle them.
pub struct VelocityController {
    /// Per-axis velocity P gain [East, North, Up], units s⁻¹ (m/s² per m/s).
    velocity_gain: Vector3<f32>,
    conditioning: SpecificForceConditioning,
    /// Outer INDI law over the 3-D specific force. Effectiveness is identity.
    inversion: IncrementalInversion<3>,
    decomposition: ThrustVectorDecomposition,
}

impl VelocityController {
    pub fn new(
        velocity_gain: Vector3<f32>,
        conditioning: SpecificForceConditioning,
        inversion: IncrementalInversion<3>,
        decomposition: ThrustVectorDecomposition,
    ) -> Self {
        Self { velocity_gain, conditioning, inversion, decomposition }
    }

    /// One outer-loop cycle. Returns the attitude setpoint for the cascade.
    pub fn step(&mut self, state: &VehicleState, setpoint: &VelocitySetpoint) -> AttitudeSetpoint {
        let v_err = Vector3::from(setpoint.linear_velocity) - state.linear_velocity();
        let a_des = self.velocity_gain.component_mul(&v_err);
        // Gravity feed-forward: f = a − g_vec, g_vec = [0,0,−g] in ENU.
        let f_des = a_des + Vector3::new(0.0, 0.0, STANDARD_GRAVITY);

        let conditioned = self.conditioning.step(state);
        // u₀ = modeled thrust (RPM), ν̂ = measured specific force (incl. disturbance).
        let tau = self
            .inversion
            .compute(&f_des, &conditioned.specific_force, &conditioned.thrust);

        // Preserve current heading; the thrust vector only fixes roll/pitch.
        let yaw = state.attitude().euler_angles().2;
        self.decomposition.compute(tau, yaw, setpoint.yaw_rate, setpoint.timestamp_us)
    }

    /// Warm-start the conditioning filters on arm / mode change.
    pub fn reset(&mut self, state: &VehicleState) {
        self.conditioning.reset(state);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::StateValidity;
    use nalgebra::{Matrix3, Vector4};

    const FS: f32 = 1000.0;
    const F_CUT: f32 = 10.0;
    const G: f32 = STANDARD_GRAVITY;
    const HOVER_OMEGA: f32 = 500.0;
    const THRUST_COEFF: f32 = 1.0 / (4.0 * HOVER_OMEGA * HOVER_OMEGA);

    fn controller() -> VelocityController {
        let conditioning = SpecificForceConditioning::new(F_CUT, FS, Vector4::repeat(THRUST_COEFF));
        let inversion = IncrementalInversion::<3>::new_uniform(
            Matrix3::identity(),
            f32::NEG_INFINITY,
            f32::INFINITY,
        )
        .unwrap();
        let decomposition = ThrustVectorDecomposition::new(0.3, 0.2, 2.5);
        VelocityController::new(Vector3::new(1.0, 1.0, 1.5), conditioning, inversion, decomposition)
    }

    /// Constant-velocity state (a = 0 → specific force is hover [0,0,g]). No motor
    /// telemetry, so the thrust model falls back to f̂ (no disturbance rejection).
    fn state_moving(vx: f32, vy: f32, vz: f32) -> VehicleState {
        VehicleState {
            linear_velocity: [vx, vy, vz],
            specific_force: [0.0, 0.0, G],
            attitude: [0.0, 0.0, 0.0, 1.0],
            valid: StateValidity::ATTITUDE.with(StateValidity::VELOCITY),
            ..VehicleState::default()
        }
    }

    fn hold() -> VelocitySetpoint {
        VelocitySetpoint { timestamp_us: 0, linear_velocity: [0.0, 0.0, 0.0], yaw_rate: 0.0 }
    }

    #[test]
    fn hover_hold_produces_level_attitude_unity_thrust() {
        let mut c = controller();
        let state = state_moving(0.0, 0.0, 0.0);
        c.reset(&state);
        let sp = c.step(&state, &hold());
        let [x, y, z, w] = sp.attitude;
        assert!(x.abs() < 1e-3 && y.abs() < 1e-3 && z.abs() < 1e-3, "expected level, got {:?}", sp.attitude);
        assert!((w - 1.0).abs() < 1e-3);
        assert!((sp.specific_thrust - 1.0).abs() < 1e-2, "thrust={}", sp.specific_thrust);
    }

    #[test]
    fn moving_east_commands_west_tilt() {
        let mut c = controller();
        let state = state_moving(1.0, 0.0, 0.0);
        c.reset(&state);
        let sp = c.step(&state, &hold());
        let [x, y, _z, _w] = sp.attitude;
        assert!(y < 0.0, "expected negative pitch (tilt west), got Im_y={y}");
        assert!(x.abs() < 1e-3, "expected no roll, got Im_x={x}");
    }

    #[test]
    fn descending_below_setpoint_raises_collective() {
        let mut c = controller();
        let state = state_moving(0.0, 0.0, -1.0);
        c.reset(&state);
        let sp = c.step(&state, &hold());
        assert!(sp.specific_thrust > 1.0, "expected thrust > 1g when sinking, got {}", sp.specific_thrust);
    }

    /// With RPM telemetry, a specific-force disturbance (wind) is rejected: the
    /// controller tilts into it even at zero velocity error.
    #[test]
    fn rejects_specific_force_disturbance_with_rpm() {
        let mut c = controller();
        // Hover hold, level, hover RPM, but accelerometer sees an eastward wind.
        let state = VehicleState {
            linear_velocity: [0.0, 0.0, 0.0],
            specific_force: [2.0, 0.0, G], // +x disturbance
            attitude: [0.0, 0.0, 0.0, 1.0],
            motor_speed: [HOVER_OMEGA; 4],
            valid: StateValidity::ATTITUDE
                .with(StateValidity::VELOCITY)
                .with(StateValidity::MOTOR_SPEED),
            ..VehicleState::default()
        };
        c.reset(&state);
        let sp = c.step(&state, &hold());
        // Tilt west (negative pitch) to push back against the eastward wind.
        let [_x, y, _z, _w] = sp.attitude;
        assert!(y < -1e-3, "expected west tilt to reject wind, got Im_y={y}");
    }
}
