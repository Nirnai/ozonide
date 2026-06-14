use crate::msgs::AngularAccelerationSetpoint;
use nalgebra::{Matrix4, Vector3, Vector4};

#[derive(Debug)]
pub struct ControlAllocator {
    /// Control effectiveness: maps actuator vector u (motor speed squared,
    /// rad²/s², motors [FR, RL, FL, RR]) to virtual control
    /// [ω̇x, ω̇y, ω̇z (rad/s²), specific thrust (g's, 1.0 = hover)].
    g: Matrix4<f32>,
    g_inv: Matrix4<f32>,
    /// Actuator limits in control-variable units (rad²/s²).
    u_min: f32,
    u_max: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum AllocatorConfigError {
    /// Swapped min and max
    SwappedMinMaxParameters,
    /// G is not invertible.
    SingularEffectiveness,
    /// G is badly conditioned, inverse entries exceed sanity bounds.
    IllConditionedEffectiveness { residual: f32 },
    /// G contains non-finite entries.
    NonFiniteEffectiveness,
}

impl ControlAllocator {
    pub fn new(g: Matrix4<f32>, u_min: f32, u_max: f32) -> Result<Self, AllocatorConfigError> {
        if u_min > u_max {
            return Err(AllocatorConfigError::SwappedMinMaxParameters);
        }
        if g.iter().any(|x| !x.is_finite()) {
            return Err(AllocatorConfigError::NonFiniteEffectiveness);
        }
        let g_inv = g
            .try_inverse()
            .ok_or(AllocatorConfigError::SingularEffectiveness)?;

        let residual = (g * g_inv - Matrix4::identity())
            .iter()
            .fold(0.0f32, |m, x| m.max(x.abs()));
        if !residual.is_finite() || residual > 1e-2 {
            return Err(AllocatorConfigError::IllConditionedEffectiveness { residual });
        }
        Ok(Self {
            g,
            g_inv,
            u_min,
            u_max,
        })
    }

    pub fn allocate(
        &self,
        angular_acceleration_setpoint: &AngularAccelerationSetpoint,
        angular_acceleration_estimate: &Vector3<f32>,
        specific_thrust_estimate: f32,
        actuator_state_estimate: &Vector4<f32>,
    ) -> Vector4<f32> {
        let virtual_control_error = Vector4::new(
            angular_acceleration_setpoint.roll_acceleration - angular_acceleration_estimate[0],
            angular_acceleration_setpoint.pitch_acceleration - angular_acceleration_estimate[1],
            angular_acceleration_setpoint.yaw_acceleration - angular_acceleration_estimate[2],
            angular_acceleration_setpoint.specific_thrust - specific_thrust_estimate,
        );
        let du = self.g_inv * virtual_control_error;
        let u = actuator_state_estimate + du;
        return u.map(|x| x.clamp(self.u_min, self.u_max));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix4, Vector3, Vector4};

    // Physical parameters the fixture is derived from (≈ your build):
    const K_T: f32 = 2.4e-6; // N/(rad/s)²
    const K_Q: f32 = 3.6e-8; // N·m/(rad/s)²
    const ARM: f32 = 0.064; // m (l_x = l_y, symmetric X)
    const I_XX: f32 = 6.0e-4; // kg·m²
    const I_YY: f32 = 6.0e-4;
    const I_ZZ: f32 = 1.1e-3;
    const MASS: f32 = 0.35; // kg
    const G0: f32 = 9.81;

    const U_MIN: f32 = 1.0e4; // rad²/s² (~Ω = 100 rad/s idle)
    const U_MAX: f32 = 1.6e6; // rad²/s² (~Ω = 1265 rad/s)

    /// Ω²-space effectiveness, derived (not hand-written) from physics.
    /// rows = [roll ω̇, pitch ω̇, yaw ω̇, specific thrust (g's)],
    /// cols = motors [FR, RL, FL, RR], u in rad²/s².
    /// Signs: roll+ = right down → left motors up; pitch+ = nose down →
    /// rear up; yaw+ = CCW → CW props (FR, RL) up.
    fn test_g() -> Matrix4<f32> {
        let r = K_T * ARM / I_XX; // ≈ 2.56e-1
        let p = K_T * ARM / I_YY;
        let y = K_Q / I_ZZ; // ≈ 3.27e-5
        let t = K_T / (MASS * G0); // ≈ 6.99e-7
        Matrix4::new(-r, r, r, -r, -p, p, -p, p, y, y, -y, -y, t, t, t, t)
    }

    /// Hover: the u₀ at which (G·u₀)[3] == 1.0 exactly, all motors equal.
    fn hover() -> Vector4<f32> {
        let u_h = (MASS * G0) / (4.0 * K_T); // ≈ 3.58e5 rad²/s²
        Vector4::repeat(u_h)
    }

    fn alloc() -> ControlAllocator {
        ControlAllocator::new(test_g(), U_MIN, U_MAX).unwrap()
    }

    fn setpoint(
        roll: f32,
        pitch: f32,
        yaw: f32,
        specific_thrust: f32,
    ) -> AngularAccelerationSetpoint {
        AngularAccelerationSetpoint {
            timestamp_us: 0,
            roll_acceleration: roll,
            pitch_acceleration: pitch,
            yaw_acceleration: yaw,
            specific_thrust,
        }
    }

    /// Relative comparison — u lives near 3.6e5, so absolute 1e-5 is meaningless in f32.
    fn assert_vec4_rel_eq(a: &Vector4<f32>, b: &Vector4<f32>, rel: f32) {
        for i in 0..4 {
            let scale = b[i].abs().max(1.0);
            assert!(
                (a[i] - b[i]).abs() <= rel * scale,
                "row {i}: {} vs {}",
                a[i],
                b[i]
            );
        }
    }

    // ---- 1. Construction ----

    #[test]
    fn swapped_limits_are_rejected() {
        assert_eq!(
            ControlAllocator::new(test_g(), U_MAX, U_MIN).unwrap_err(),
            AllocatorConfigError::SwappedMinMaxParameters
        );
    }

    #[test]
    fn non_finite_g_is_rejected() {
        let mut g = test_g();
        g[(2, 1)] = f32::NAN;
        assert_eq!(
            ControlAllocator::new(g, U_MIN, U_MAX).unwrap_err(),
            AllocatorConfigError::NonFiniteEffectiveness
        );
    }

    #[test]
    fn inverse_is_actually_the_inverse() {
        let a = alloc();
        let identity = a.g * a.g_inv;
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                // f32 through a badly row-scaled matrix: 1e-3 is realistic.
                assert!(
                    (identity[(i, j)] - expected).abs() < 1e-3,
                    "({i},{j}): {}",
                    identity[(i, j)]
                );
            }
        }
    }

    #[test]
    fn singular_g_is_rejected() {
        let mut g = test_g();
        g.set_row(1, &g.row(0).clone_owned()); // duplicate row → singular
        assert_eq!(
            ControlAllocator::new(g, U_MIN, U_MAX).unwrap_err(),
            AllocatorConfigError::SingularEffectiveness
        );
    }

    // ---- 2. The fixed-point property ----

    #[test]
    fn zero_error_returns_actuator_state_unchanged() {
        let a = alloc();
        let h = hover();
        // All four axes compare against measurements now. "No error" means:
        // demanded ω̇ equals measured ω̇, demanded thrust equals measured thrust.
        let sp = setpoint(0.0, 0.0, 0.0, 1.0);
        let measured_omega_dot = Vector3::zeros();
        let measured_thrust = 1.0; // g's, consistent with hovering

        let out = a.allocate(&sp, &measured_omega_dot, measured_thrust, &h);
        assert_vec4_rel_eq(&out, &h, 1e-5);
    }

    // ---- 3. The increment property (the core INDI invariant) ----

    #[test]
    fn increment_produces_exactly_the_demanded_virtual_control_change() {
        let a = alloc();
        let h = hover();
        // Measurements deliberately inconsistent with G·u₀ — the disturbance
        // case — on ALL four axes now (e.g. measured 1.1 g in ground effect).
        let measured = Vector3::new(1.5, -2.0, 0.3);
        let measured_thrust = 1.1;
        let sp = setpoint(10.0, -5.0, 2.0, 1.0);

        let out = a.allocate(&sp, &measured, measured_thrust, &h);
        let achieved = test_g() * (out - h);

        assert!((achieved[0] - (10.0 - 1.5)).abs() < 1e-2);
        assert!((achieved[1] - (-5.0 - -2.0)).abs() < 1e-2);
        assert!((achieved[2] - (2.0 - 0.3)).abs() < 1e-2);
        assert!((achieved[3] - (1.0 - 1.1)).abs() < 1e-4); // thrust commanded DOWN
    }

    // ---- 4. Per-axis sign/direction tests ----
    // Comparisons are against hover()[i], not 0.5: "increase" now means
    // "above the hover Ω² this motor started at".

    #[test]
    fn positive_roll_demand_raises_left_motors() {
        let a = alloc();
        let h = hover();
        let u = a.allocate(&setpoint(50.0, 0.0, 0.0, 1.0), &Vector3::zeros(), 1.0, &h); // [FR, RL, FL, RR]
        assert!(u[1] > h[1] && u[2] > h[2], "RL, FL should increase");
        assert!(u[0] < h[0] && u[3] < h[3], "FR, RR should decrease");
    }

    #[test]
    fn positive_pitch_demand_raises_rear_motors() {
        let a = alloc();
        let h = hover();
        let u = a.allocate(&setpoint(0.0, 50.0, 0.0, 1.0), &Vector3::zeros(), 1.0, &h);
        assert!(u[1] > h[1] && u[3] > h[3], "RL, RR should increase");
        assert!(u[0] < h[0] && u[2] < h[2], "FR, FL should decrease");
    }

    #[test]
    fn positive_yaw_demand_raises_cw_motors() {
        let a = alloc();
        let h = hover();
        let u = a.allocate(&setpoint(0.0, 0.0, 20.0, 1.0), &Vector3::zeros(), 1.0, &h);
        assert!(u[0] > h[0] && u[1] > h[1], "FR, RL (CW) should increase");
        assert!(u[2] < h[2] && u[3] < h[3], "FL, RR (CCW) should decrease");
    }

    #[test]
    fn thrust_increase_raises_all_motors_equally() {
        let a = alloc();
        let h = hover();
        // Demand 1.05 g while measuring 1.0 g → +0.05 g increment, collective.
        let u = a.allocate(&setpoint(0.0, 0.0, 0.0, 1.05), &Vector3::zeros(), 1.0, &h);
        assert!(u.iter().all(|&x| x > h[0]));
        let spread =
            u.iter().cloned().fold(f32::MIN, f32::max) - u.iter().cloned().fold(f32::MAX, f32::min);
        // Relative: u is O(3.6e5); absolute 1e-5 would be below f32 ULP here.
        assert!(
            spread < 1e-3 * h[0],
            "symmetric G: collective stays collective"
        );
    }

    // ---- 5. Saturation ----

    #[test]
    fn saturating_demand_clamps_and_stays_finite() {
        let a = alloc();
        // Near the ceiling, then demand an absurd roll acceleration.
        let high = Vector4::repeat(0.9 * U_MAX);
        let out = a.allocate(
            &setpoint(5000.0, 0.0, 0.0, (test_g() * high)[3]),
            &Vector3::zeros(),
            (test_g() * high)[3],
            &high,
        );
        for &x in out.iter() {
            assert!(x.is_finite());
            assert!((U_MIN..=U_MAX).contains(&x));
        }
        assert!(
            out.iter().any(|&x| x == U_MAX || x == U_MIN),
            "demand this large must pin at least one motor"
        );
    }
}
