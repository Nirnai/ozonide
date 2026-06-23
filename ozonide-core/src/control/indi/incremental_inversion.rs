use nalgebra::allocator::Allocator;
use nalgebra::{Const, DefaultAllocator, DimMin, OMatrix, OVector};

/// Generic incremental nonlinear dynamic inversion (INDI) law.
///
/// Implements the one operation shared by every INDI loop, for an
/// `N`-dimensional virtual control inverted onto `N` actuators:
///
/// ```text
/// Δν = ν_desired − ν̂            (virtual-control error; measured ν̂ fed back)
/// Δu = B⁻¹ · Δν                 (B: control effectiveness, ν = B·u)
/// u  = u₀ + Δu                  (increment around the current input)
/// u  = clamp(u, u_min, u_max)   (per-channel saturation)
/// ```
///
/// The two features that make this *incremental* — feeding back the measured
/// virtual control `ν̂`, and incrementing around the current input `u₀` — are
/// exactly what separates INDI from plain (static) control allocation.
///
/// Instantiate once per loop with that loop's effectiveness matrix and limits.
/// The inner rate loop uses `N = 4` (`[αx, αy, αz, specific thrust] → 4 motors`);
/// the outer loop instantiates its own `B` over specific forces.
///
/// `B` is square: this is exact inversion, not redundant allocation. For a
/// quad the 4 virtual controls map to exactly 4 motors.
#[derive(Debug)]
pub struct IncrementalInversion<const N: usize>
where
    DefaultAllocator: Allocator<Const<N>, Const<N>> + Allocator<Const<N>>,
{
    /// Control effectiveness `B`: `ν = B·u`. Identified on the bench.
    effectiveness: OMatrix<f32, Const<N>, Const<N>>,
    effectiveness_inv: OMatrix<f32, Const<N>, Const<N>>,
    /// Per-channel saturation limits, same units as `u`.
    u_min: OVector<f32, Const<N>>,
    u_max: OVector<f32, Const<N>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum IncrementalInversionError {
    /// A channel's lower limit exceeds its upper limit.
    SwappedLimits,
    /// `B` is not invertible.
    SingularEffectiveness,
    /// `B` is badly conditioned: inverse entries exceed sanity bounds.
    IllConditionedEffectiveness { residual: f32 },
    /// `B` contains non-finite entries.
    NonFiniteEffectiveness,
}

impl<const N: usize> IncrementalInversion<N>
where
    Const<N>: DimMin<Const<N>, Output = Const<N>>,
    DefaultAllocator: Allocator<Const<N>, Const<N>> + Allocator<Const<N>>,
{
    /// Build from an effectiveness matrix and per-channel limits. Validates that
    /// `B` is finite, invertible, and well-conditioned, and that the limits are
    /// ordered. The inverse is computed once here, never on the hot path.
    pub fn new(
        effectiveness: OMatrix<f32, Const<N>, Const<N>>,
        u_min: OVector<f32, Const<N>>,
        u_max: OVector<f32, Const<N>>,
    ) -> Result<Self, IncrementalInversionError> {
        if u_min.iter().zip(u_max.iter()).any(|(lo, hi)| lo > hi) {
            return Err(IncrementalInversionError::SwappedLimits);
        }
        if effectiveness.iter().any(|x| !x.is_finite()) {
            return Err(IncrementalInversionError::NonFiniteEffectiveness);
        }
        let effectiveness_inv = effectiveness
            .clone()
            .try_inverse()
            .ok_or(IncrementalInversionError::SingularEffectiveness)?;

        let residual = (&effectiveness * &effectiveness_inv
            - OMatrix::<f32, Const<N>, Const<N>>::identity())
        .iter()
        .fold(0.0f32, |m, x| m.max(x.abs()));
        if !residual.is_finite() || residual > 1e-2 {
            return Err(IncrementalInversionError::IllConditionedEffectiveness { residual });
        }

        Ok(Self { effectiveness, effectiveness_inv, u_min, u_max })
    }

    /// Convenience constructor when every channel shares the same limits
    /// (e.g. the rate loop, where all four motors have identical Ω² bounds).
    pub fn new_uniform(
        effectiveness: OMatrix<f32, Const<N>, Const<N>>,
        u_min: f32,
        u_max: f32,
    ) -> Result<Self, IncrementalInversionError> {
        Self::new(
            effectiveness,
            OVector::<f32, Const<N>>::repeat(u_min),
            OVector::<f32, Const<N>>::repeat(u_max),
        )
    }

    /// The INDI increment law. Returns the saturated actuator command
    /// `u = clamp(u₀ + B⁻¹·(ν_desired − ν̂))`.
    ///
    /// - `virtual_desired` — ν_desired, the virtual control the outer law wants.
    /// - `virtual_measured` — ν̂, the current virtual control estimated from
    ///   sensors (filtered angular acceleration, measured specific thrust, …).
    /// - `current_input` — u₀, the actuator state the increment is taken around.
    pub fn compute(
        &self,
        virtual_desired: &OVector<f32, Const<N>>,
        virtual_measured: &OVector<f32, Const<N>>,
        current_input: &OVector<f32, Const<N>>,
    ) -> OVector<f32, Const<N>> {
        let error = virtual_desired - virtual_measured;
        let du = &self.effectiveness_inv * error;
        let u = current_input + du;
        u.zip_zip_map(&self.u_min, &self.u_max, |x, lo, hi| x.clamp(lo, hi))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Matrix4, Vector4};

    // Physical parameters the fixture is derived from (≈ the test build):
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
    /// rows = [roll α, pitch α, yaw α, specific thrust (g's)],
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

    fn inversion() -> IncrementalInversion<4> {
        IncrementalInversion::new_uniform(test_g(), U_MIN, U_MAX).unwrap()
    }

    /// Pack a virtual-control vector [roll α, pitch α, yaw α, specific thrust].
    fn nu(roll: f32, pitch: f32, yaw: f32, specific_thrust: f32) -> Vector4<f32> {
        Vector4::new(roll, pitch, yaw, specific_thrust)
    }

    /// Relative comparison — u lives near 3.6e5, so absolute 1e-5 is meaningless in f32.
    fn assert_vec4_rel_eq(a: &Vector4<f32>, b: &Vector4<f32>, rel: f32) {
        for i in 0..4 {
            let scale = b[i].abs().max(1.0);
            assert!((a[i] - b[i]).abs() <= rel * scale, "row {i}: {} vs {}", a[i], b[i]);
        }
    }

    // ---- 1. Construction ----

    #[test]
    fn swapped_limits_are_rejected() {
        assert_eq!(
            IncrementalInversion::new_uniform(test_g(), U_MAX, U_MIN).unwrap_err(),
            IncrementalInversionError::SwappedLimits
        );
    }

    #[test]
    fn non_finite_g_is_rejected() {
        let mut g = test_g();
        g[(2, 1)] = f32::NAN;
        assert_eq!(
            IncrementalInversion::new_uniform(g, U_MIN, U_MAX).unwrap_err(),
            IncrementalInversionError::NonFiniteEffectiveness
        );
    }

    #[test]
    fn inverse_is_actually_the_inverse() {
        let a = inversion();
        let identity = a.effectiveness * a.effectiveness_inv;
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { 1.0 } else { 0.0 };
                // f32 through a badly row-scaled matrix: 1e-3 is realistic.
                assert!((identity[(i, j)] - expected).abs() < 1e-3, "({i},{j}): {}", identity[(i, j)]);
            }
        }
    }

    #[test]
    fn singular_g_is_rejected() {
        let mut g = test_g();
        g.set_row(1, &g.row(0).clone_owned()); // duplicate row → singular
        assert_eq!(
            IncrementalInversion::new_uniform(g, U_MIN, U_MAX).unwrap_err(),
            IncrementalInversionError::SingularEffectiveness
        );
    }

    // ---- 2. The fixed-point property ----

    #[test]
    fn zero_error_returns_current_input_unchanged() {
        let a = inversion();
        let h = hover();
        // "No error": demanded ν equals measured ν on all four axes.
        let desired = nu(0.0, 0.0, 0.0, 1.0);
        let measured = nu(0.0, 0.0, 0.0, 1.0); // hovering: 0 α, 1 g thrust
        let out = a.compute(&desired, &measured, &h);
        assert_vec4_rel_eq(&out, &h, 1e-5);
    }

    // ---- 3. The increment property (the core INDI invariant) ----

    #[test]
    fn increment_produces_exactly_the_demanded_virtual_control_change() {
        let a = inversion();
        let h = hover();
        // Measurements deliberately inconsistent with G·u₀ — the disturbance
        // case — on ALL four axes (e.g. measured 1.1 g in ground effect).
        let measured = nu(1.5, -2.0, 0.3, 1.1);
        let desired = nu(10.0, -5.0, 2.0, 1.0);

        let out = a.compute(&desired, &measured, &h);
        let achieved = test_g() * (out - h);

        assert!((achieved[0] - (10.0 - 1.5)).abs() < 1e-2);
        assert!((achieved[1] - (-5.0 - -2.0)).abs() < 1e-2);
        assert!((achieved[2] - (2.0 - 0.3)).abs() < 1e-2);
        assert!((achieved[3] - (1.0 - 1.1)).abs() < 1e-4); // thrust commanded DOWN
    }

    // ---- 4. Per-axis sign/direction tests ----
    // Comparisons are against hover()[i]: "increase" means above the hover Ω²
    // this motor started at.

    #[test]
    fn positive_roll_demand_raises_left_motors() {
        let a = inversion();
        let h = hover();
        let u = a.compute(&nu(50.0, 0.0, 0.0, 1.0), &nu(0.0, 0.0, 0.0, 1.0), &h); // [FR, RL, FL, RR]
        assert!(u[1] > h[1] && u[2] > h[2], "RL, FL should increase");
        assert!(u[0] < h[0] && u[3] < h[3], "FR, RR should decrease");
    }

    #[test]
    fn positive_pitch_demand_raises_rear_motors() {
        let a = inversion();
        let h = hover();
        let u = a.compute(&nu(0.0, 50.0, 0.0, 1.0), &nu(0.0, 0.0, 0.0, 1.0), &h);
        assert!(u[1] > h[1] && u[3] > h[3], "RL, RR should increase");
        assert!(u[0] < h[0] && u[2] < h[2], "FR, FL should decrease");
    }

    #[test]
    fn positive_yaw_demand_raises_cw_motors() {
        let a = inversion();
        let h = hover();
        let u = a.compute(&nu(0.0, 0.0, 20.0, 1.0), &nu(0.0, 0.0, 0.0, 1.0), &h);
        assert!(u[0] > h[0] && u[1] > h[1], "FR, RL (CW) should increase");
        assert!(u[2] < h[2] && u[3] < h[3], "FL, RR (CCW) should decrease");
    }

    #[test]
    fn thrust_increase_raises_all_motors_equally() {
        let a = inversion();
        let h = hover();
        // Demand 1.05 g while measuring 1.0 g → +0.05 g increment, collective.
        let u = a.compute(&nu(0.0, 0.0, 0.0, 1.05), &nu(0.0, 0.0, 0.0, 1.0), &h);
        assert!(u.iter().all(|&x| x > h[0]));
        let spread =
            u.iter().cloned().fold(f32::MIN, f32::max) - u.iter().cloned().fold(f32::MAX, f32::min);
        // Relative: u is O(3.6e5); absolute 1e-5 would be below f32 ULP here.
        assert!(spread < 1e-3 * h[0], "symmetric G: collective stays collective");
    }

    // ---- 5. Saturation ----

    #[test]
    fn saturating_demand_clamps_and_stays_finite() {
        let a = inversion();
        // Near the ceiling, then demand an absurd roll acceleration.
        let high = Vector4::repeat(0.9 * U_MAX);
        let thrust_high = (test_g() * high)[3];
        let out = a.compute(
            &nu(5000.0, 0.0, 0.0, thrust_high),
            &nu(0.0, 0.0, 0.0, thrust_high),
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
