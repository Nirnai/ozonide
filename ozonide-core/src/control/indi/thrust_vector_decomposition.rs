use libm::{cosf, sinf};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

use crate::msgs::{AttitudeSetpoint, STANDARD_GRAVITY};

/// Geometric output map for the outer INDI loop: turns a desired thrust
/// specific-force vector into an [`AttitudeSetpoint`].
///
/// This is the outer-loop analogue of the inner loop's inverse actuator model —
/// both convert the INDI law's output into the form the next stage consumes.
/// Here the thrust vector `τ` (world ENU, m/s², pointing along desired body-z)
/// is split into:
/// - **collective specific thrust** = `|τ| / g` (g's), what the rate loop wants;
/// - **attitude**: the rotation whose body-z (FLU thrust axis) aligns with `τ̂`,
///   with heading set by the desired yaw (a free DOF the thrust vector doesn't
///   constrain).
///
/// The decomposition is globally exact — no small-angle assumption — which is
/// the whole reason the thrust-vector form is preferred over an angle Jacobian.
/// Saturation (max tilt, thrust bounds) lives here, because the INDI clamp runs
/// with infinite limits.
pub struct ThrustVectorDecomposition {
    max_tilt: f32,
    sin_max_tilt: f32,
    cos_max_tilt: f32,
    /// Collective specific-thrust bounds, g's.
    min_thrust: f32,
    max_thrust: f32,
}

impl ThrustVectorDecomposition {
    /// `max_tilt` in rad; `min_thrust`/`max_thrust` are collective specific
    /// thrust bounds in g's (hover ≈ 1.0).
    pub fn new(max_tilt: f32, min_thrust: f32, max_thrust: f32) -> Self {
        Self {
            max_tilt,
            sin_max_tilt: sinf(max_tilt),
            cos_max_tilt: cosf(max_tilt),
            min_thrust,
            max_thrust,
        }
    }

    /// Decompose the world-frame thrust specific-force vector `tau` (m/s²) into
    /// an attitude setpoint, preserving `yaw` (rad) as the heading and passing
    /// `yaw_rate` straight through.
    pub fn compute(
        &self,
        tau: Vector3<f32>,
        yaw: f32,
        yaw_rate: f32,
        timestamp_us: u64,
    ) -> AttitudeSetpoint {
        let magnitude = tau.norm();
        let specific_thrust = (magnitude / STANDARD_GRAVITY).clamp(self.min_thrust, self.max_thrust);

        // Desired body-z (thrust axis) in world. Degenerate near-zero thrust
        // (free-fall command) → keep level rather than spin on undefined direction.
        let mut z_des = if magnitude > 1e-3 {
            tau / magnitude
        } else {
            Vector3::z()
        };

        // Tilt saturation: cos(tilt) = z_des·ẑ_world = z_des.z. tilt exceeds the
        // limit iff that drops below cos(max_tilt); project onto the cone, keeping
        // azimuth. This also catches z_des.z < 0 (thrust pointing down → flip).
        if z_des[2] < self.cos_max_tilt {
            let horizontal = Vector3::new(z_des[0], z_des[1], 0.0);
            let h_norm = horizontal.norm();
            z_des = if h_norm > 1e-6 {
                horizontal / h_norm * self.sin_max_tilt + Vector3::z() * self.cos_max_tilt
            } else {
                Vector3::z()
            };
        }

        // Build the rotation: body-z = z_des, heading set by yaw. Standard
        // geometric construction (x_c = world heading projection).
        let x_c = Vector3::new(cosf(yaw), sinf(yaw), 0.0);
        let mut y_b = z_des.cross(&x_c);
        let y_norm = y_b.norm();
        y_b = if y_norm > 1e-6 { y_b / y_norm } else { Vector3::y() };
        let x_b = y_b.cross(&z_des);

        let basis = Matrix3::from_columns(&[x_b, y_b, z_des]);
        let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(basis));
        let c = q.coords;

        AttitudeSetpoint {
            timestamp_us,
            attitude: [c.x, c.y, c.z, c.w],
            yaw_rate,
            specific_thrust,
        }
    }

    pub fn max_tilt(&self) -> f32 {
        self.max_tilt
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Quaternion, UnitQuaternion};

    const G: f32 = STANDARD_GRAVITY;

    fn decomp() -> ThrustVectorDecomposition {
        ThrustVectorDecomposition::new(0.3, 0.2, 2.5)
    }

    fn quat(sp: &AttitudeSetpoint) -> UnitQuaternion<f32> {
        let [x, y, z, w] = sp.attitude;
        UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
    }

    #[test]
    fn hover_thrust_gives_level_attitude_unity_thrust() {
        let sp = decomp().compute(Vector3::new(0.0, 0.0, G), 0.0, 0.0, 0);
        let [x, y, z, w] = sp.attitude;
        assert!(x.abs() < 1e-5 && y.abs() < 1e-5 && z.abs() < 1e-5, "expected identity, got {:?}", sp.attitude);
        assert!((w - 1.0).abs() < 1e-5);
        assert!((sp.specific_thrust - 1.0).abs() < 1e-4, "thrust={}", sp.specific_thrust);
    }

    #[test]
    fn east_thrust_produces_positive_pitch() {
        // tau tilted east (+x) → body-z tilts east → positive pitch.
        let sp = decomp().compute(Vector3::new(2.0, 0.0, G), 0.0, 0.0, 0);
        let [x, y, _z, _w] = sp.attitude;
        assert!(y > 0.0, "expected positive pitch (Im_y>0), got {y}");
        assert!(x.abs() < 1e-4, "expected no roll, got {x}");
    }

    #[test]
    fn north_thrust_produces_negative_roll() {
        let sp = decomp().compute(Vector3::new(0.0, 2.0, G), 0.0, 0.0, 0);
        let [x, y, _z, _w] = sp.attitude;
        assert!(x < 0.0, "expected negative roll (Im_x<0), got {x}");
        assert!(y.abs() < 1e-4, "expected no pitch, got {y}");
    }

    #[test]
    fn tilt_is_saturated_to_max() {
        // Huge horizontal component → tilt pinned at max_tilt.
        let sp = decomp().compute(Vector3::new(100.0, 0.0, G), 0.0, 0.0, 0);
        let z_body = quat(&sp) * Vector3::z();
        let cos_tilt = z_body[2];
        assert!((cos_tilt - 0.3_f32.cos()).abs() < 1e-3, "tilt not clamped: cos_tilt={cos_tilt}");
    }

    #[test]
    fn collective_thrust_is_clamped() {
        let sp_hi = decomp().compute(Vector3::new(0.0, 0.0, 100.0 * G), 0.0, 0.0, 0);
        assert!((sp_hi.specific_thrust - 2.5).abs() < 1e-4, "high thrust not clamped: {}", sp_hi.specific_thrust);
        let sp_lo = decomp().compute(Vector3::new(0.0, 0.0, 0.5), 0.0, 0.0, 0);
        assert!((sp_lo.specific_thrust - 0.2).abs() < 1e-4, "low thrust not clamped: {}", sp_lo.specific_thrust);
    }

    #[test]
    fn yaw_sets_heading() {
        use core::f32::consts::FRAC_PI_2;
        let sp = decomp().compute(Vector3::new(0.0, 0.0, G), FRAC_PI_2, 0.0, 0);
        // Pure yaw of 90° → q = Rz(π/2): Im_z = sin(π/4).
        let [x, y, z, w] = sp.attitude;
        assert!(x.abs() < 1e-4 && y.abs() < 1e-4, "expected pure yaw, got {:?}", sp.attitude);
        assert!((z - (FRAC_PI_2 / 2.0).sin()).abs() < 1e-4, "z={z}");
        assert!((w - (FRAC_PI_2 / 2.0).cos()).abs() < 1e-4, "w={w}");
    }

    #[test]
    fn yaw_rate_passes_through() {
        let sp = decomp().compute(Vector3::new(0.0, 0.0, G), 0.0, 0.7, 0);
        assert!((sp.yaw_rate - 0.7).abs() < 1e-6);
    }
}
