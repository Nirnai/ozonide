use nalgebra::{Quaternion, UnitQuaternion};

use crate::msgs::{AngularVelocitySetpoint, AttitudeSetpoint, VehicleState};

use super::{Pid, PidGains};

/// Extract (roll, pitch) Euler angles from a `VehicleState` attitude quaternion.
fn roll_pitch(state: &VehicleState) -> (f32, f32) {
    let [x, y, z, w] = state.attitude;
    let q = UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));
    let (roll, pitch, _) = q.euler_angles();
    (roll, pitch)
}

/// PID gains for the roll and pitch angle outer loops.
///
/// Yaw has no angle loop (gravity gives no yaw reference), so only roll and
/// pitch are tuned here. Yaw is commanded as a rate directly by the pilot or
/// a higher-level controller and passes through unchanged.
#[derive(Clone, Copy)]
pub struct AttitudeGains {
    pub roll_angle_control_gains: PidGains,
    pub pitch_angle_control_gains: PidGains,
}

/// Outer-loop angle controller.
///
/// Computes angular rate setpoints from attitude angle errors. This is the outer
/// loop of a cascaded attitude controller — its outputs feed directly into
/// [`AngularVelocityController`](crate::control::AngularVelocityController) as setpoints:
///
/// ```text
/// AttitudeSetpoint → AttitudeController → AngularVelocitySetpoint → AngularVelocityController → corrections → mixer
///                           ↑
///                      (this struct)
/// ```
///
/// ## Update law (roll and pitch)
///
/// Given angle setpoint `θ_cmd` and estimated angle `θ` from the complementary filter:
///
/// ```text
/// error        = θ_cmd − θ
/// rate_setpoint = Kp·error + Ki·∫error·dt + Kd·d(error)/dt
/// ```
///
/// The output `rate_setpoint` is in rad/s and is bounded by `output_limit` in
/// [`AttitudeGains`]. A typical value is `π/2` rad/s (90°/s) to prevent the
/// inner rate loop from being commanded beyond its authority.
///
/// ## Yaw pass-through
///
/// Yaw cannot be corrected by the accelerometer, so there is no absolute yaw
/// reference in the complementary filter. Yaw is therefore commanded as a rate
/// (`yaw_rate` from [`AttitudeSetpoint`]) and passed directly to the rate loop
/// without a PID outer loop.
///
/// ## Derivative term
///
/// The D term on the angle loop is often left at zero (kd = 0). The inner rate
/// loop already provides angular velocity damping via its proportional term, so
/// the D term in the outer loop would be redundant and amplifies noise. Include
/// it only if the angle response is underdamped with kd = 0.
#[derive(Default)]
pub struct AttitudeController {
    roll_angle_controller: Pid,
    pitch_angle_controller: Pid,
}

impl AttitudeController {
    /// Advances the roll and pitch angle loops by one time step and returns rate setpoints.
    ///
    /// # Arguments
    /// * `setpoint` — desired attitude and yaw rate from the pilot or outer guidance loop
    /// * `estimate` — current attitude estimate; only `roll` and `pitch` are used
    /// * `gains`    — PID gains per axis (passed per-call for gain scheduling)
    /// * `dt`       — time elapsed since the previous call (s); returns zero setpoints if ≤ 0
    pub fn update(
        &mut self,
        state: &VehicleState,
        setpoint: &AttitudeSetpoint,
        gains: &AttitudeGains,
        dt: f32,
    ) -> AngularVelocitySetpoint {
        let (roll, pitch) = roll_pitch(state);
        let roll_error  = setpoint.roll  - roll;
        let pitch_error = setpoint.pitch - pitch;

        AngularVelocitySetpoint {
            roll_rate: self.roll_angle_controller.update(roll_error, &gains.roll_angle_control_gains, dt),
            pitch_rate: self.pitch_angle_controller.update(pitch_error, &gains.pitch_angle_control_gains, dt),
            yaw_rate: setpoint.yaw_rate,
        }
    }

    /// Resets all integrators and derivative state to zero.
    ///
    /// Call this on arming or mode transitions to prevent stale integral
    /// wind-up from affecting the first active control output.
    pub fn reset(&mut self) {
        self.roll_angle_controller = Pid::default();
        self.pitch_angle_controller = Pid::default();
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

    fn uniform_gains(kp: f32) -> AttitudeGains {
        AttitudeGains { roll_angle_control_gains: p_gains(kp), pitch_angle_control_gains: p_gains(kp) }
    }

    fn setpoint(roll: f32, pitch: f32, yaw_rate: f32) -> AttitudeSetpoint {
        AttitudeSetpoint { roll, pitch, yaw_rate, thrust: 0.5 }
    }

    fn state_at(roll: f32, pitch: f32) -> VehicleState {
        let q = UnitQuaternion::from_euler_angles(roll, pitch, 0.0_f32);
        let c = q.coords;
        VehicleState { attitude: [c.x, c.y, c.z, c.w], ..VehicleState::default() }
    }

    #[test]
    fn zero_error_produces_zero_rate_setpoints() {
        let mut ctrl = AttitudeController::default();
        let gains = uniform_gains(5.0);
        // Setpoint matches estimate → error = 0 → rate setpoints = 0.
        let out = ctrl.update(&state_at(0.3, -0.2), &setpoint(0.3, -0.2, 0.1), &gains, 0.01);
        assert_approx(out.roll_rate, 0.0);
        assert_approx(out.pitch_rate, 0.0);
    }

    #[test]
    fn proportional_rate_setpoint_scales_with_error() {
        let mut ctrl = AttitudeController::default();
        let gains = uniform_gains(3.0);
        // Estimate at zero; setpoints give known errors.
        let out = ctrl.update(&state_at(0.0, 0.0), &setpoint(0.2, -0.1, 0.0), &gains, 0.01);
        assert_approx(out.roll_rate,  3.0 *  0.2);
        assert_approx(out.pitch_rate, 3.0 * -0.1);
    }

    #[test]
    fn axes_are_independent() {
        // Error on roll only; pitch rate setpoint must be zero.
        let mut ctrl = AttitudeController::default();
        let gains = AttitudeGains { roll_angle_control_gains: p_gains(4.0), pitch_angle_control_gains: p_gains(0.0) };
        let out = ctrl.update(&state_at(0.0, 0.0), &setpoint(1.0, 0.5, 0.0), &gains, 0.01);
        assert_approx(out.roll_rate,  4.0);
        assert_approx(out.pitch_rate, 0.0);
    }

    #[test]
    fn yaw_rate_is_passed_through_unchanged() {
        let mut ctrl = AttitudeController::default();
        let gains = uniform_gains(1.0);
        let out = ctrl.update(&state_at(0.0, 0.0), &setpoint(0.0, 0.0, 0.75), &gains, 0.01);
        assert_approx(out.yaw_rate, 0.75);
    }

    #[test]
    fn zero_dt_returns_zero_rate_setpoints() {
        let mut ctrl = AttitudeController::default();
        let gains = uniform_gains(10.0);
        let out = ctrl.update(&state_at(0.0, 0.0), &setpoint(1.0, 1.0, 0.0), &gains, 0.0);
        assert_approx(out.roll_rate, 0.0);
        assert_approx(out.pitch_rate, 0.0);
    }

    #[test]
    fn output_limit_caps_rate_setpoint() {
        let mut ctrl = AttitudeController::default();
        // kp=10, error=1 → unbounded output = 10 rad/s; limit to π/2 ≈ 1.5708.
        let limit = core::f32::consts::FRAC_PI_2;
        let gains = AttitudeGains {
            roll_angle_control_gains:  PidGains { kp: 10.0, ki: 0.0, kd: 0.0, integral_limit: 0.0, output_limit: limit },
            pitch_angle_control_gains: PidGains { kp: 10.0, ki: 0.0, kd: 0.0, integral_limit: 0.0, output_limit: limit },
        };
        let out = ctrl.update(&state_at(0.0, 0.0), &setpoint(1.0, -1.0, 0.0), &gains, 0.01);
        assert_approx(out.roll_rate,   limit);
        assert_approx(out.pitch_rate, -limit);
    }

    #[test]
    fn reset_clears_integrator() {
        let mut ctrl = AttitudeController::default();
        let gains = AttitudeGains {
            roll_angle_control_gains:  PidGains { kp: 0.0, ki: 1.0, kd: 0.0, integral_limit: 10.0, output_limit: f32::MAX },
            pitch_angle_control_gains: PidGains { kp: 0.0, ki: 1.0, kd: 0.0, integral_limit: 10.0, output_limit: f32::MAX },
        };
        for _ in 0..5 {
            ctrl.update(&state_at(0.0, 0.0), &setpoint(1.0, 1.0, 0.0), &gains, 0.1);
        }
        ctrl.reset();
        let out = ctrl.update(&state_at(0.0, 0.0), &setpoint(0.0, 0.0, 0.0),  &gains, 0.1);
        assert_approx(out.roll_rate, 0.0);
        assert_approx(out.pitch_rate, 0.0);
    }
}
