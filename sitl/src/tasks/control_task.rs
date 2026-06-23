use nalgebra::{Matrix3, Vector3};

use ozonide_core::config::VehicleConfig;
use ozonide_core::control::indi::{
    AngularRateController, CascadedController, IncrementalInversion, InputSignalConditioning,
    InverseActuatorModel, SpecificForceConditioning, ThrustVectorDecomposition, VelocityController,
};
use ozonide_core::msgs::VelocitySetpoint;

use crate::setpoint_simulated::SetpointSimulated;

/// Control loop sample rate (Hz). Must match the IMU publish rate.
const FS: f32 = 1000.0;

/// Rate-loop signal conditioning cutoff (Hz). Trades α noise for loop phase margin.
const F_CUT: f32 = 20.0;

/// Outer-loop specific-force conditioning cutoff (Hz). Lower than the rate path:
/// the velocity loop is slow, so heavy accelerometer filtering is cheap here.
const F_CUT_VELOCITY: f32 = 10.0;

/// Inner-loop rate-P gains [roll, pitch, yaw], s⁻¹.
const RATE_GAIN: [f32; 3] = [10.0, 10.0, 5.0];

/// Outer-loop attitude-P gains [roll, pitch, yaw], s⁻¹.
/// Must be well below the inner-loop bandwidth to maintain cascade stability.
const ATTITUDE_GAIN: [f32; 3] = [4.0, 4.0, 2.0];

/// Velocity-loop P gains [East, North, Up], s⁻¹ (m/s² per m/s). Kept below the
/// attitude loop (4 s⁻¹) so the cascade stays separated. Horizontal bumped to 2
/// for tighter gust recovery.
const VELOCITY_GAIN: [f32; 3] = [2.0, 2.0, 2.0];

/// Maximum tilt angle commanded by the velocity controller (rad ≈ 17°).
const MAX_TILT: f32 = 0.3;

/// Collective specific-thrust bounds, g's.
const MIN_THRUST: f32 = 0.2;
const MAX_THRUST: f32 = 2.5;

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

    let rate = AngularRateController::new(
        conditioning,
        inversion,
        output_map,
        Vector3::from(RATE_GAIN),
    );

    // Outer INDI force loop: identity effectiveness (specific-force units), so
    // the INDI clamp is disabled (±∞) and saturation lives in the decomposition.
    // u₀ comes from the RPM thrust model — the thrust row (g's per Ω²) of the
    // effectiveness matrix, so f̂ − u₀ isolates the specific-force disturbance.
    let thrust_coeff = cfg.effectiveness_matrix().row(3).transpose();
    let force_conditioning = SpecificForceConditioning::new(F_CUT_VELOCITY, FS, thrust_coeff);
    let force_inversion = IncrementalInversion::<3>::new_uniform(
        Matrix3::identity(),
        f32::NEG_INFINITY,
        f32::INFINITY,
    )
    .expect("identity effectiveness is always valid");
    let decomposition = ThrustVectorDecomposition::new(MAX_TILT, MIN_THRUST, MAX_THRUST);
    let velocity = VelocityController::new(
        Vector3::from(VELOCITY_GAIN),
        force_conditioning,
        force_inversion,
        decomposition,
    );

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
