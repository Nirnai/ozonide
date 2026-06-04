use nalgebra::Vector3;

pub const GRAVITATIONAL_CONSTANT: f64 = 9.80665; // m/s², standard gravity (ISO 80000-3)

pub struct VehicleParameters {
    pub mass: f64,
    pub inertia: Vector3<f64>,
    pub actuator_positions: [Vector3<f64>; 4],
    pub actuator_spin_directions: [f64; 4],
    pub k_drag_lin: f64,                       // translational drag
    pub k_drag_rot: f64,                       // rotational drag
    pub k_spring: f64,                         // ground contact stiffness
    pub k_damp_ground: f64,                    // ground contact damping
    pub k_friction_ground: f64,                // ground friction
    pub tau_wind: f64,                         // corralation time (e.g. 1s gust)
    pub sigma_wind_force: f64,
    pub sigma_wind_torque: f64  

}

impl Default for VehicleParameters {
    fn default() -> Self {
        let mass = 0.5;
        let arm = 0.15_f64; 
        let h = arm / std::f64::consts::SQRT_2;
        let sigma_force = 0.027 * mass * GRAVITATIONAL_CONSTANT;
        let sigma_torque = sigma_force * Vector3::new(h, h, 0.0).norm() / 10.0;
        Self {
            mass: mass,
            inertia: Vector3::new(4e-3, 4e-3, 7e-3), // Ixx, Iyy, Izz kg·m²
            actuator_positions: [
                Vector3::new(h, -h, 0.0),  // FR (CW)
                Vector3::new(-h, h, 0.0),  // RL (CW)
                Vector3::new(h, h, 0.0),   // FL (CCW)
                Vector3::new(-h, -h, 0.0)  // RR (CCW)
            ],
            actuator_spin_directions: [1.0, 1.0, -1.0, -1.0], // CW=+1, CCW=-1
            k_drag_lin: 0.1,        // N·s/m
            k_drag_rot: 0.001,      // N·m·s/rad
            k_spring: 2000.0,       // N/m
            k_damp_ground: 100.0,   // N·s/m
            k_friction_ground: 50.0, // N·s/m
            // 0.2s — rapid turbulence, wind changes direction frequently; hard for the controller
            // 1.0s — outdoor gust, persists long enough to require sustained corrective action; good default
            // 5.0s — slow, steady wind bias; tests the integrator's ability to reject a constant offset
            tau_wind: 1.0,
            sigma_wind_force: sigma_force,
            sigma_wind_torque: sigma_torque 
        }
    }
}
