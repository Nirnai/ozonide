use nalgebra::Vector3;

use ozonide_core::config::VehicleConfig;
use ozonide_core::control::indi::{
    AngularVelocityController, CascadedController, IncrementalInversion,
    InputSignalConditioning, InverseActuatorModel, VelocityController,
};
use ozonide_core::msgs::VelocitySetpoint;

use crate::setpoint_simulated::SetpointSimulated;

/// Control loop sample rate (Hz). Must match the IMU publish rate.
const FS: f32 = 1000.0;

/// Signal conditioning cutoff (Hz). Trades ω̇ noise for loop phase margin.
const F_CUT: f32 = 20.0;

/// Inner-loop rate-P gains [roll, pitch, yaw], s⁻¹.
const RATE_GAIN: [f32; 3] = [10.0, 10.0, 5.0];

/// Outer-loop attitude-P gains [roll, pitch, yaw], s⁻¹.
/// Must be well below the inner-loop bandwidth to maintain cascade stability.
const ATTITUDE_GAIN: [f32; 3] = [4.0, 4.0, 2.0];

/// Velocity-loop horizontal P gain (rad / (m/s)).
const VELOCITY_GAIN: f32 = 0.3;

/// Maximum tilt angle commanded by the velocity controller (rad ≈ 17°).
const MAX_TILT: f32 = 0.3;

/// Vertical velocity P gain (g / (m/s)).
const VELOCITY_GAIN_Z: f32 = 0.5;

/// Hover setpoint: zero velocity in all axes.
const HOVER_SETPOINT: VelocitySetpoint = VelocitySetpoint {
    timestamp_us: 0,
    linear_velocity: [0.0, 0.0, 0.0],
    yaw_rate: 0.0,
};

pub fn make_controller() -> CascadedController {
    let cfg = VehicleConfig::default();
    let (u_min, u_max) = cfg.actuator_limits();
    let (c0, c1) = cfg.motor_model_coefficients();
    let v = cfg.battery_nominal_voltage;

    let conditioning = InputSignalConditioning::new(F_CUT, FS, cfg.motor_time_constant);
    let inversion = IncrementalInversion::<4>::new_uniform(cfg.effectiveness_matrix(), u_min, u_max)
        .expect("VehicleConfig::default yields a valid effectiveness matrix");
    let output_map = core::array::from_fn(|_| {
        InverseActuatorModel::new(c0, c1, v, v, cfg.idle_throttle)
            .expect("VehicleConfig::default yields valid motor model parameters")
    });

    let rate = AngularVelocityController::new(
        conditioning,
        inversion,
        output_map,
        Vector3::from(RATE_GAIN),
    );

    let velocity = VelocityController::new(VELOCITY_GAIN, MAX_TILT, VELOCITY_GAIN_Z);

    CascadedController::new(velocity, Vector3::from(ATTITUDE_GAIN), rate)
}

pub fn make_setpoint_source() -> SetpointSimulated {
    SetpointSimulated::new(HOVER_SETPOINT)
}

#[embassy_executor::task]
pub async fn control_task(
    controller: &'static mut CascadedController,
    setpoint_source: &'static mut SetpointSimulated,
) {
    ozonide_core::tasks::control_task(controller, setpoint_source).await;
}
