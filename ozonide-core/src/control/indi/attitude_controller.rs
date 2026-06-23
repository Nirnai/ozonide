use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::msgs::{AngularVelocitySetpoint, AttitudeSetpoint, VehicleState};

/// Stateless quaternion attitude P-controller.
///
/// Computes body-frame angular rate commands from the quaternion attitude error.
/// Call [`compute`](Self::compute) once per control cycle — no internal state.
///
/// # Error formulation
///
/// ```text
/// q_err = q_meas⁻¹ ⊗ q_des              (error expressed in body frame)
/// φ_err = 2 · sgn(q_err.w) · Im(q_err)  (rotation vector, rad, body frame)
/// ω_cmd = Kp · φ_err
/// ```
///
/// The `sgn(q_err.w)` flip handles the unit-quaternion double cover: `q` and
/// `−q` represent the same rotation; without it the controller would take the
/// "long arc" (> π) for half the attitude space.
///
/// For small errors, `Im(q_err) ≈ φ/2`, so `2·Im(q_err) ≈ φ` recovers the
/// rotation vector in radians and `Kp` has units of `s⁻¹`.
pub struct AttitudeController {
    /// Proportional gain per axis `[roll, pitch, yaw]`, units: s⁻¹.
    pub gain: Vector3<f32>,
}

impl AttitudeController {
    pub fn new(gain: Vector3<f32>) -> Self {
        Self { gain }
    }

    /// Compute the angular rate setpoint for one control cycle.
    pub fn compute(&self, state: &VehicleState, setpoint: &AttitudeSetpoint) -> AngularVelocitySetpoint {
        let q_meas = state.attitude();
        let [x, y, z, w] = setpoint.attitude;
        let q_des = UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));

        let q_err = q_meas.inverse() * q_des;

        // Short-arc: choose the hemisphere where w ≥ 0.
        let sign = if q_err.w >= 0.0 { 1.0_f32 } else { -1.0_f32 };
        let phi_err = sign * 2.0 * q_err.vector();

        AngularVelocitySetpoint {
            timestamp_us: setpoint.timestamp_us,
            roll_rate:       self.gain[0] * phi_err[0],
            pitch_rate:      self.gain[1] * phi_err[1],
            yaw_rate:        self.gain[2] * phi_err[2] + setpoint.yaw_rate,
            specific_thrust: setpoint.specific_thrust,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    fn level() -> [f32; 4] {
        [0.0, 0.0, 0.0, 1.0]
    }

    fn quat_to_sp(q: UnitQuaternion<f32>) -> [f32; 4] {
        let c = q.coords;
        [c.x, c.y, c.z, c.w]
    }

    fn state_at(q: UnitQuaternion<f32>) -> VehicleState {
        let c = q.coords;
        VehicleState { attitude: [c.x, c.y, c.z, c.w], ..VehicleState::default() }
    }

    fn ctrl(kp: f32) -> AttitudeController {
        AttitudeController::new(Vector3::from_element(kp))
    }

    fn sp(attitude: [f32; 4]) -> AttitudeSetpoint {
        AttitudeSetpoint { timestamp_us: 0, attitude, yaw_rate: 0.0, specific_thrust: 1.0 }
    }

    #[test]
    fn zero_error_produces_zero_rates() {
        let q = UnitQuaternion::from_euler_angles(0.3, -0.2, 1.0);
        let out = ctrl(5.0).compute(&state_at(q), &sp(quat_to_sp(q)));
        assert!(out.roll_rate.abs()  < 1e-5);
        assert!(out.pitch_rate.abs() < 1e-5);
        assert!(out.yaw_rate.abs()   < 1e-5);
    }

    #[test]
    fn level_setpoint_from_level_state_produces_zero_rates() {
        let out = ctrl(5.0).compute(&state_at(UnitQuaternion::identity()), &sp(level()));
        assert!(out.roll_rate.abs()  < 1e-5);
        assert!(out.pitch_rate.abs() < 1e-5);
        assert!(out.yaw_rate.abs()   < 1e-5);
    }

    #[test]
    fn small_roll_error_scales_with_gain() {
        let angle = 0.1_f32;
        let q_des = UnitQuaternion::from_euler_angles(angle, 0.0, 0.0);
        let out = ctrl(4.0).compute(&state_at(UnitQuaternion::identity()), &sp(quat_to_sp(q_des)));
        // For small angles: phi_err[0] ≈ angle, so roll_rate ≈ Kp * angle.
        assert!((out.roll_rate  - 4.0 * angle).abs() < 0.01);
        assert!(out.pitch_rate.abs() < 1e-4);
        assert!(out.yaw_rate.abs()   < 1e-4);
    }

    #[test]
    fn double_cover_negative_quaternion_gives_same_command() {
        let q = UnitQuaternion::from_euler_angles(0.2, 0.1, 0.5);
        let c = q.coords;
        let sp_pos = sp([c.x,  c.y,  c.z,  c.w]);
        let sp_neg = sp([-c.x, -c.y, -c.z, -c.w]);
        let out_pos = ctrl(3.0).compute(&state_at(UnitQuaternion::identity()), &sp_pos);
        let out_neg = ctrl(3.0).compute(&state_at(UnitQuaternion::identity()), &sp_neg);
        assert!((out_pos.roll_rate  - out_neg.roll_rate).abs()  < 1e-5);
        assert!((out_pos.pitch_rate - out_neg.pitch_rate).abs() < 1e-5);
        assert!((out_pos.yaw_rate   - out_neg.yaw_rate).abs()   < 1e-5);
    }

    #[test]
    fn specific_thrust_passes_through() {
        let mut setpoint = sp(level());
        setpoint.specific_thrust = 0.7;
        let out = ctrl(1.0).compute(&state_at(UnitQuaternion::identity()), &setpoint);
        assert!((out.specific_thrust - 0.7).abs() < 1e-6);
    }
}
