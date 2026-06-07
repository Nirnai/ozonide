use ozonide_core::control::{AttitudeGains, AngularVelocityGains, CascadedPidController, PidGains};
use ozonide_core::msgs::AttitudeSetpoint;

use crate::setpoint_simulated::SetpointSimulated;

/// Hover setpoint: wings level, zero yaw rate, exact hover thrust.
///
/// Derived from simulator parameters (parameters.rs / actuator_model.rs):
///   mass = 0.5 kg,  g = 9.80665 m/s²,  k_t = 5.5e-7 N/(rad/s)²,  ω_max = 3000 rad/s
///   ω_hover   = sqrt(mass · g / (4 · k_t)) = sqrt(2_228_784.09) ≈ 1492.91 rad/s
///   throttle  = ω_hover / ω_max             ≈ 0.4976
const HOVER_SETPOINT: AttitudeSetpoint = AttitudeSetpoint {
    roll: 0.0,
    pitch: 0.0,
    yaw_rate: 0.0,
    thrust: 0.4976,
};

/// Outer-loop (angle) gains.
///
/// Output is a rate setpoint in rad/s, capped at ±3 rad/s (~170 °/s).
/// D term is left at zero — the inner rate loop already provides damping.
fn attitude_gains() -> AttitudeGains {
    let axis = PidGains {
        kp: 4.0,
        ki: 0.0,
        kd: 0.0,
        integral_limit: 0.0,
        output_limit: 3.0,
    };
    AttitudeGains {
        roll_angle_control_gains: axis,
        pitch_angle_control_gains: axis,
    }
}

/// Inner-loop (rate) gains.
///
/// Output is a normalised torque in [-1, 1] fed to the mixer.
/// Yaw uses lower gains — higher inertia, weaker actuator authority.
fn angular_velocity_gains() -> AngularVelocityGains {
    let roll_pitch = PidGains {
        kp: 0.15,
        ki: 0.05,
        kd: 0.05,
        integral_limit: 0.3,
        output_limit: 0.5,
    };
    let yaw = PidGains {
        kp: 0.10,
        ki: 0.02,
        kd: 0.00,
        integral_limit: 0.3,
        output_limit: 0.5,
    };
    AngularVelocityGains {
        roll_rate_control_gains: roll_pitch,
        pitch_rate_control_gains: roll_pitch,
        yaw_rate_control_gains: yaw,
    }
}

pub fn make_controller() -> CascadedPidController {
    CascadedPidController::new(attitude_gains(), angular_velocity_gains())
}

pub fn make_setpoint_source() -> SetpointSimulated {
    SetpointSimulated::new(HOVER_SETPOINT)
}

#[embassy_executor::task]
pub async fn control_task(
    controller: &'static mut CascadedPidController,
    setpoint_source: &'static mut SetpointSimulated,
) {
    ozonide_core::tasks::control_task(controller, setpoint_source).await;
}
