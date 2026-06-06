//! Physical parameters describing the quadrotor airframe and environment.

use nalgebra::Vector3;

/// Standard gravitational acceleration (m/s²), ISO 80000-3.
pub const GRAVITATIONAL_CONSTANT: f64 = 9.80665;

/// Physical constants that fully describe the vehicle and its environment.
///
/// All quantities use SI units. The [`Default`] values correspond to a
/// 500 g, 5-inch-class racing quadrotor with a 150 mm motor-to-motor arm.
pub struct VehicleParameters {
    /// Vehicle mass (kg).
    pub mass: f64,
    /// Principal moments of inertia \[Ixx, Iyy, Izz\] in the body frame (kg·m²).
    /// Off-diagonal terms are neglected (symmetric airframe assumed).
    pub inertia: Vector3<f64>,
    /// Motor positions in the body frame (m), indexed \[FR, RL, FL, RR\].
    /// Used to compute roll, pitch, and yaw moments via the moment-arm cross product.
    pub actuator_positions: [Vector3<f64>; 4],
    /// Spin direction of each motor: `+1.0` = CW, `−1.0` = CCW (viewed from above).
    /// Opposite diagonal pairs share the same sign so yaw reaction torques cancel at hover.
    pub actuator_spin_directions: [f64; 4],
    /// Translational drag coefficient (N·s/m).
    /// Applied as `F_drag = −k_drag_lin · v` in the world frame.
    pub k_drag_lin: f64,
    /// Rotational drag coefficient (N·m·s/rad).
    /// Applied as `τ_drag = −k_drag_rot · ω` in the body frame.
    pub k_drag_rot: f64,
    /// Ground contact spring stiffness (N/m).
    /// Part of the spring-damper model that prevents the vehicle penetrating the ground.
    pub k_spring: f64,
    /// Ground contact damping coefficient (N·s/m).
    pub k_damp_ground: f64,
    /// Ground friction coefficient (N·s/m).
    /// Applied as a velocity-proportional XY drag force while `z ≤ 0`.
    pub k_friction_ground: f64,
    /// Wind gust correlation time τ (s) for the Ornstein-Uhlenbeck disturbance model.
    ///
    /// Controls how long a gust persists before reverting to zero:
    /// - 0.2 s — rapid turbulence, hard for a controller
    /// - 1.0 s — outdoor gust (default)
    /// - 5.0 s — slow steady wind bias
    pub tau_wind: f64,
    /// Standard deviation of the wind force disturbance per axis (N).
    /// Default: `0.027 · mass · g ≈ 0.13 N`.
    pub sigma_wind_force: f64,
    /// Standard deviation of the wind torque disturbance per axis (N·m).
    pub sigma_wind_torque: f64,
}

impl Default for VehicleParameters {
    fn default() -> Self {
        let mass = 0.5;
        let arm = 0.15_f64;
        let h = arm / std::f64::consts::SQRT_2;
        let sigma_force = 0.027 * mass * GRAVITATIONAL_CONSTANT;
        let sigma_torque = sigma_force * Vector3::new(h, h, 0.0).norm() / 10.0;
        Self {
            mass,
            inertia: Vector3::new(4e-3, 4e-3, 7e-3), // Ixx, Iyy, Izz kg·m²
            actuator_positions: [
                Vector3::new(h, -h, 0.0),  // FR (CW)
                Vector3::new(-h, h, 0.0),  // RL (CW)
                Vector3::new(h, h, 0.0),   // FL (CCW)
                Vector3::new(-h, -h, 0.0), // RR (CCW)
            ],
            actuator_spin_directions: [1.0, 1.0, -1.0, -1.0], // CW=+1, CCW=-1
            k_drag_lin: 0.1,         // N·s/m
            k_drag_rot: 0.001,       // N·m·s/rad
            k_spring: 2000.0,        // N/m
            k_damp_ground: 100.0,    // N·s/m
            k_friction_ground: 50.0, // N·s/m
            tau_wind: 1.0,
            sigma_wind_force: sigma_force,
            sigma_wind_torque: sigma_torque,
        }
    }
}
