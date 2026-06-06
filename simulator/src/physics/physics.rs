//! RK4 rigid body integrator for the quadrotor dynamics.
//!
//! The public entry point is [`step`], which advances the full vehicle state
//! by one time step. Internal helpers compute the state derivative ([`derivative`])
//! and perform partial integration ([`integrate`]) for the four RK4 stages.
//!
//! ## Forces and torques modelled
//!
//! | Source | Frame | Expression |
//! |--------|-------|------------|
//! | Motor thrust + drag | Body → World | actuator wrench via moment arms |
//! | Gravity | World | `−g·m·ẑ` |
//! | Translational drag | World | `−k_drag_lin · v` |
//! | Rotational drag | Body | `−k_drag_rot · ω` |
//! | Ground spring-damper | World Z | prevents terrain penetration |
//! | Ground friction | World XY | velocity damping when `z ≤ 0` |
//! | External disturbances | World | Gaussian or OU wind gusts |

use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};
use rand::Rng;

use super::parameters::{VehicleParameters, GRAVITATIONAL_CONSTANT};
use super::state::{VehicleState, VehicleStateDot};
use crate::models::{ActuatorModel, DisturbanceModel};


/// Advances the vehicle state by `dt` seconds using a classical 4th-order Runge-Kutta scheme.
///
/// The disturbance wrench is sampled **once** before the four derivative evaluations
/// and treated as constant across all RK4 stages. This is both physically correct
/// (external wind does not change within a single step) and required for the
/// Ornstein-Uhlenbeck model, whose internal state must advance exactly once per step.
///
/// # Arguments
/// * `throttle`          — normalised motor commands \[FR, RL, FL, RR\] in `[0, 1]`
/// * `state`             — vehicle state at the start of the step
/// * `params`            — airframe physical parameters
/// * `actuator_model`    — motor and propeller aerodynamic model
/// * `disturbance_model` — wind/gust model (OU state is mutated here)
/// * `rng`               — random number generator for stochastic disturbances
/// * `dt`                — integration time step (s); typically 0.25 ms at 4 kHz
pub fn step(
    throttle: Vector4<f64>,
    state: &VehicleState,
    params: &VehicleParameters,
    actuator_model: &ActuatorModel,
    disturbance_model: &mut DisturbanceModel,
    rng: &mut impl Rng,
    dt: f64,
) -> VehicleState {
    let disturbance_wrench = disturbance_model.generate(params, rng, dt);
    // let disturbances = random_disturbances(rng, params);
    let k1 = derivative(throttle, state, params, actuator_model, disturbance_wrench);
    let k2 = derivative(
        throttle,
        &integrate(state, &k1, dt / 2.0),
        params,
        actuator_model,
        disturbance_wrench,
    );
    let k3 = derivative(
        throttle,
        &integrate(state, &k2, dt / 2.0),
        params,
        actuator_model,
        disturbance_wrench,
    );
    let k4 = derivative(
        throttle,
        &integrate(state, &k3, dt),
        params,
        actuator_model,
        disturbance_wrench,
    );
    let combined_state_dot = k1 + k2 * 2.0 + k3 * 2.0 + k4;
    integrate(state, &combined_state_dot, dt / 6.0)
}

fn integrate(state: &VehicleState, state_dot: &VehicleStateDot, dt: f64) -> VehicleState {
    VehicleState {
        position: state.position + state_dot.linear_velocity * dt,
        attitude: UnitQuaternion::from_quaternion(
            state.attitude.into_inner() + state_dot.attitude_rate * dt,
        ),
        linear_velocity: state.linear_velocity + state_dot.linear_acceleration * dt,
        angular_velocity: state.angular_velocity + state_dot.angular_acceleration * dt,
        motor_angular_velocity: state.motor_angular_velocity
            + state_dot.motor_angular_acceleration * dt,
    }
}

fn derivative(
    throttle: Vector4<f64>,
    state: &VehicleState,
    params: &VehicleParameters,
    actuator_model: &ActuatorModel,
    disturbance: (Vector3<f64>, Vector3<f64>)
) -> VehicleStateDot {
    let (actuator_force_body, actuator_torque_body) =
        wrench(state.motor_angular_velocity, actuator_model, params);
    // let (disturbance_force, disturbance_torque) = external_disturbance_wrench(rng, params);
    let (disturbance_force, disturbance_torque) = disturbance;
    let ground_friction_forces = ground_friction(state.position.z, state.linear_velocity, params.k_friction_ground);
    let force_world = state.attitude * actuator_force_body
        + gravity(params.mass)
        + translational_drag(state.linear_velocity, params.k_drag_lin)
        + Vector3::new(
            ground_friction_forces.x,
            ground_friction_forces.y,
            ground_contact_forces(
                state.position.z,
                state.linear_velocity.z,
                params.k_spring,
                params.k_damp_ground,
            ),
        )
        + disturbance_force;
    let torque_body = actuator_torque_body
        + rotational_drag(state.angular_velocity, params.k_drag_rot)
        + disturbance_torque;

    let linear_acceleration = force_world / params.mass;
    let i_omega = params.inertia.component_mul(&state.angular_velocity);
    let angular_acceleration =
        (torque_body - state.angular_velocity.cross(&i_omega)).component_div(&params.inertia);
    let attitude_rate =
        0.5 * state.attitude.quaternion() * Quaternion::from_parts(0.0, state.angular_velocity);
    let motor_angular_acceleration = Vector4::new(
        actuator_model.omega_dot(state.motor_angular_velocity[0], throttle[0]),
        actuator_model.omega_dot(state.motor_angular_velocity[1], throttle[1]),
        actuator_model.omega_dot(state.motor_angular_velocity[2], throttle[2]),
        actuator_model.omega_dot(state.motor_angular_velocity[3], throttle[3]),
    );
    VehicleStateDot {
        linear_velocity: state.linear_velocity,
        attitude_rate,
        linear_acceleration,
        angular_acceleration,
        motor_angular_acceleration,
    }
}

fn gravity(mass: f64) -> Vector3<f64> {
    Vector3::new(0.0, 0.0, -GRAVITATIONAL_CONSTANT * mass)
}

fn ground_contact_forces(
    position_z: f64,
    velocity_z: f64,
    k_spring: f64,
    k_damp_ground: f64,
) -> f64 {
    (-k_spring * position_z - k_damp_ground * velocity_z).max(0.0)
}

fn ground_friction(
    position_z: f64,
    velocity: Vector3<f64>,
    k_friction_ground: f64,
) -> Vector3<f64> {
    if position_z <= 0.0{
        -k_friction_ground * velocity
    } else {
        Vector3::zeros()
    }
}

fn translational_drag(velocity: Vector3<f64>, k_drag_lin: f64) -> Vector3<f64> {
    -k_drag_lin * velocity
}

fn rotational_drag(angular_velocity: Vector3<f64>, k_drag_rot: f64) -> Vector3<f64> {
    -k_drag_rot * angular_velocity
}

fn wrench(
    motor_rpms: Vector4<f64>,
    actuator_model: &ActuatorModel,
    vehicle_parameters: &VehicleParameters,
) -> (Vector3<f64>, Vector3<f64>) {
    let mut total_force = Vector3::zeros();
    let mut total_torque = Vector3::zeros();
    for i in 0..motor_rpms.len() {
        let (thrust_i, torque_i) = actuator_model.wrench(motor_rpms[i]);
        total_force += Vector3::new(0.0, 0.0, thrust_i);
        total_torque +=
            vehicle_parameters.actuator_positions[i].cross(&Vector3::new(0.0, 0.0, thrust_i));
        total_torque += vehicle_parameters.actuator_spin_directions[i] * torque_i * Vector3::z();
    }
    (total_force, total_torque)
}


