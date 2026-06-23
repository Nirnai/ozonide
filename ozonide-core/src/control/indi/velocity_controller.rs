use nalgebra::{UnitQuaternion, Vector3};

use crate::msgs::{AttitudeSetpoint, VelocitySetpoint, VehicleState};

/// Stateless horizontal velocity P-controller (outer loop).
///
/// Maps a world-frame (ENU) velocity error to a desired tilt [`AttitudeSetpoint`]
/// while preserving the vehicle's current yaw heading.
///
/// # Sign convention (FLU body, ENU world)
///
/// ```text
/// Ry(pitch) * z_body = [sin(pitch), 0, cos(pitch)]
///   → pitch > 0  →  East (+x) thrust
///
/// Rx(roll)  * z_body = [0, -sin(roll), cos(roll)]
///   → roll < 0   →  North (+y) thrust
/// ```
///
/// Therefore:  `pitch_des = Kp * v_err_east`,  `roll_des = −Kp * v_err_north`.
pub struct VelocityController {
    /// Horizontal proportional gain (s⁻¹). Maps m/s velocity error to rad tilt.
    pub gain: f32,
    /// Maximum tilt angle (rad). Saturates roll and pitch independently.
    pub max_tilt: f32,
    /// Vertical velocity proportional gain. Maps m/s climb error to g's of extra thrust.
    pub gain_z: f32,
}

impl VelocityController {
    pub fn new(gain: f32, max_tilt: f32, gain_z: f32) -> Self {
        Self { gain, max_tilt, gain_z }
    }

    /// Compute the attitude setpoint for one control cycle.
    pub fn compute(&self, state: &VehicleState, setpoint: &VelocitySetpoint) -> AttitudeSetpoint {
        let v_meas = state.linear_velocity();
        let v_des = Vector3::from(setpoint.linear_velocity);
        let v_err = v_des - v_meas;

        let pitch_des = (self.gain * v_err[0]).clamp(-self.max_tilt, self.max_tilt);
        let roll_des  = (-self.gain * v_err[1]).clamp(-self.max_tilt, self.max_tilt);

        // Preserve current yaw; apply desired tilt in the yaw-aligned body frame.
        let yaw = state.attitude().euler_angles().2;
        let q_des = UnitQuaternion::from_euler_angles(0.0, 0.0, yaw)
            * UnitQuaternion::from_euler_angles(0.0, pitch_des, 0.0)
            * UnitQuaternion::from_euler_angles(roll_des, 0.0, 0.0);

        let specific_thrust = (1.0 + self.gain_z * v_err[2]).clamp(0.2, 2.5);

        let c = q_des.coords;
        AttitudeSetpoint {
            timestamp_us: setpoint.timestamp_us,
            attitude: [c.x, c.y, c.z, c.w],
            yaw_rate: setpoint.yaw_rate,
            specific_thrust,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::{StateValidity, VehicleState};

    fn ctrl() -> VelocityController {
        VelocityController::new(0.3, 0.5, 0.5)
    }

    fn state_with_vel(vx: f32, vy: f32, vz: f32) -> VehicleState {
        VehicleState {
            linear_velocity: [vx, vy, vz],
            valid: StateValidity::ATTITUDE.with(StateValidity::VELOCITY),
            attitude: [0.0, 0.0, 0.0, 1.0],
            ..VehicleState::default()
        }
    }

    fn sp(vx: f32, vy: f32, vz: f32) -> VelocitySetpoint {
        VelocitySetpoint { timestamp_us: 0, linear_velocity: [vx, vy, vz], yaw_rate: 0.0 }
    }

    #[test]
    fn zero_error_produces_level_attitude() {
        let out = ctrl().compute(&state_with_vel(1.0, 2.0, 0.0), &sp(1.0, 2.0, 0.0));
        let [x, y, z, w] = out.attitude;
        // q should be identity (no tilt, no yaw)
        assert!(x.abs() < 1e-4, "x={x}");
        assert!(y.abs() < 1e-4, "y={y}");
        assert!(z.abs() < 1e-4, "z={z}");
        assert!((w - 1.0).abs() < 1e-4, "w={w}");
        assert!((out.specific_thrust - 1.0).abs() < 1e-4);
    }

    #[test]
    fn east_velocity_error_produces_positive_pitch() {
        // Want to go East, currently stationary → need to pitch forward (positive pitch).
        let out = ctrl().compute(&state_with_vel(0.0, 0.0, 0.0), &sp(1.0, 0.0, 0.0));
        // Extract pitch from quaternion: q = Ry(pitch) → y-component of Im(q) = sin(pitch/2)
        // For small pitch_des ≈ 0.3 rad, pitch/2 ≈ 0.15 → Im_y ≈ 0.149
        let [x, y, _z, _w] = out.attitude;
        assert!(y > 0.0, "expected positive pitch tilt, got Im_y={y}");
        assert!(x.abs() < 1e-4, "expected no roll, got Im_x={x}");
    }

    #[test]
    fn north_velocity_error_produces_negative_roll() {
        // Want to go North, currently stationary → need negative roll.
        let out = ctrl().compute(&state_with_vel(0.0, 0.0, 0.0), &sp(0.0, 1.0, 0.0));
        let [x, y, _z, _w] = out.attitude;
        assert!(x < 0.0, "expected negative roll tilt, got Im_x={x}");
        assert!(y.abs() < 1e-4, "expected no pitch, got Im_y={y}");
    }

    #[test]
    fn tilt_is_clamped() {
        // Very large velocity error should be saturated.
        let out = ctrl().compute(&state_with_vel(0.0, 0.0, 0.0), &sp(100.0, 0.0, 0.0));
        let [_x, y, _z, w] = out.attitude;
        let pitch = 2.0 * y.atan2(w);
        assert!(pitch <= 0.5 + 1e-4, "pitch={pitch} exceeded max_tilt");
    }

    #[test]
    fn yaw_rate_passes_through() {
        let mut s = sp(0.0, 0.0, 0.0);
        s.yaw_rate = 0.5;
        let out = ctrl().compute(&state_with_vel(0.0, 0.0, 0.0), &s);
        assert!((out.yaw_rate - 0.5).abs() < 1e-6);
    }
}
