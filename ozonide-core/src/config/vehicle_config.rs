use nalgebra::Matrix4;
use serde::{Deserialize, Serialize};

use crate::msgs::STANDARD_GRAVITY;

/// Physical parameters that fully describe the vehicle.
///
/// Motor order throughout: `[FR, RL, FL, RR]`.
#[derive(Clone, Serialize, Deserialize)]
pub struct VehicleConfig {
    /// Vehicle mass (kg).
    pub mass: f32,

    /// Principal moments of inertia `[Ixx, Iyy, Izz]` (kg·m²).
    /// Off-diagonal terms are neglected (symmetric airframe assumed).
    pub inertia: [f32; 3],

    /// Motor positions in the body frame (m), `[FR, RL, FL, RR]`.
    /// Body frame is FLU: x forward, y left, z up.
    pub motor_positions: [[f32; 3]; 4],

    /// Spin direction of each motor: `+1.0` = CW, `−1.0` = CCW (viewed from above).
    pub motor_spin_directions: [f32; 4],

    /// Thrust coefficient (N/(rad/s)²): `F = k_thrust · ω²`.
    pub k_thrust: f32,

    /// Reaction torque coefficient (N·m/(rad/s)²): `Q = k_torque · ω²`.
    pub k_torque: f32,

    /// Maximum rotor speed (rad/s). Maps throttle = 1.0 → Ω_cmd = omega_max.
    pub motor_omega_max: f32,

    /// Motor first-order lag time constant (s). Captures ESC latency and rotor inertia.
    pub motor_time_constant: f32,

    /// Nominal battery voltage (V). Used by the inverse actuator model when no
    /// live battery measurement is available.
    pub battery_nominal_voltage: f32,

    /// Minimum throttle keeping motors spinning when armed (normalized, [0, 1)).
    pub idle_throttle: f32,
}

impl VehicleConfig {
    /// Control effectiveness matrix G mapping actuator vector u = [Ω₀², Ω₁², Ω₂², Ω₃²]
    /// (rad²/s²) to virtual control [α_roll, α_pitch, α_yaw (rad/s²), specific_thrust (g's)].
    ///
    /// Derived from the moment-arm cross products and the blade-element force laws:
    ///
    /// ```text
    /// τ_roll  = Σ  pos_y[i] · k_thrust · ω_i²
    /// τ_pitch = Σ −pos_x[i] · k_thrust · ω_i²
    /// τ_yaw   = Σ  spin[i]  · k_torque · ω_i²
    /// a_z     = Σ  k_thrust · ω_i² / (mass · g)
    /// ```
    pub fn effectiveness_matrix(&self) -> Matrix4<f32> {
        let [ixx, iyy, izz] = self.inertia;
        let mg = self.mass * STANDARD_GRAVITY;

        let mut g = Matrix4::zeros();
        for i in 0..4 {
            let [px, py, _] = self.motor_positions[i];
            let spin = self.motor_spin_directions[i];
            g[(0, i)] =  py * self.k_thrust / ixx;   // roll
            g[(1, i)] = -px * self.k_thrust / iyy;   // pitch
            g[(2, i)] =  spin * self.k_torque / izz; // yaw
            g[(3, i)] =  self.k_thrust / mg;          // specific thrust
        }
        g
    }

    /// Actuator limits in Ω² (rad²/s²) for the control allocator.
    ///
    /// Returns `(u_min, u_max)` where `u_min` is the idle floor and `u_max`
    /// corresponds to full throttle.
    pub fn actuator_limits(&self) -> (f32, f32) {
        let omega_idle = self.idle_throttle * self.motor_omega_max;
        let u_min = omega_idle * omega_idle;
        let u_max = self.motor_omega_max * self.motor_omega_max;
        (u_min, u_max)
    }

    /// Inverse actuator model parameters `(c0, c1)` for `InverseActuatorModel::new`.
    ///
    /// For the linear throttle→speed model used in simulation:
    ///   `Ω = omega_max · throttle`  →  `throttle = Ω / omega_max`
    /// which is `c0 = 0`, `c1 = omega_max`.
    ///
    /// On a real vehicle, `c0` and `c1` are identified per-motor on the bench.
    /// Replace these values with bench measurements before flying.
    pub fn motor_model_coefficients(&self) -> (f32, f32) {
        (0.0, self.motor_omega_max)
    }
}

impl Default for VehicleConfig {
    /// Default parameters for a 500 g, 5-inch-class quadrotor matching the
    /// built-in simulator plant (`simulator::physics::parameters::VehicleParameters`).
    fn default() -> Self {
        // Motor-to-motor arm = 0.15 m → per-axis offset h = arm / √2
        let h = libm::sqrtf(0.15_f32 * 0.15_f32 / 2.0);
        Self {
            mass: 0.5,
            inertia: [4e-3, 4e-3, 7e-3],
            motor_positions: [
                [ h, -h, 0.0], // FR (CW)
                [-h,  h, 0.0], // RL (CW)
                [ h,  h, 0.0], // FL (CCW)
                [-h, -h, 0.0], // RR (CCW)
            ],
            motor_spin_directions: [1.0, 1.0, -1.0, -1.0],
            k_thrust: 5.5e-7,
            k_torque: 1.1e-8,
            motor_omega_max: 3000.0,
            motor_time_constant: 0.015,
            battery_nominal_voltage: 11.1,
            idle_throttle: 0.05,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> VehicleConfig {
        VehicleConfig::default()
    }

    /// At hover, all four motors run at the same speed Ω_hover = sqrt(mass·g / (4·k_t)).
    /// G · [Ω_hover²; Ω_hover²; Ω_hover²; Ω_hover²] must give [0, 0, 0, 1.0].
    #[test]
    fn hover_maps_to_unit_specific_thrust_zero_moments() {
        let cfg = default_config();
        let g = cfg.effectiveness_matrix();

        let omega_hover =
            libm::sqrtf(cfg.mass * STANDARD_GRAVITY / (4.0 * cfg.k_thrust));
        let u_hover = nalgebra::Vector4::from_element(omega_hover * omega_hover);

        let virtual_control = g * u_hover;

        assert!(virtual_control[0].abs() < 1e-6, "roll accel at hover: {}", virtual_control[0]);
        assert!(virtual_control[1].abs() < 1e-6, "pitch accel at hover: {}", virtual_control[1]);
        assert!(virtual_control[2].abs() < 1e-6, "yaw accel at hover: {}", virtual_control[2]);
        assert!(
            (virtual_control[3] - 1.0).abs() < 1e-5,
            "specific thrust at hover: {}",
            virtual_control[3]
        );
    }

    /// Increasing FR and RR (right-side motors, y < 0) while decreasing FL and RL
    /// produces positive roll (right side down in FLU convention).
    #[test]
    fn roll_axis_sign_is_correct() {
        let cfg = default_config();
        let g = cfg.effectiveness_matrix();

        // Δu positive on FR(0) and RR(3) (y < 0), negative on RL(1) and FL(2) (y > 0)
        let du = nalgebra::Vector4::new(1.0, -1.0, -1.0, 1.0);
        let virtual_control = g * du;
        assert!(virtual_control[0] < 0.0, "expected negative roll (left roll), got {}", virtual_control[0]);
    }

    /// Increasing front motors (FR, FL) while decreasing rear (RL, RR) pitches nose up.
    #[test]
    fn pitch_axis_sign_is_correct() {
        let cfg = default_config();
        let g = cfg.effectiveness_matrix();

        // FR(0) and FL(2) have px > 0 (forward), RL(1) and RR(3) have px < 0 (rear)
        // Increasing front motors → more pitch-up moment
        let du = nalgebra::Vector4::new(1.0, -1.0, 1.0, -1.0);
        let virtual_control = g * du;
        assert!(virtual_control[1] < 0.0, "expected negative pitch (nose up in FLU), got {}", virtual_control[1]);
    }

    /// CW motors (FR, RL) create positive yaw reaction; CCW (FL, RR) create negative.
    #[test]
    fn yaw_axis_sign_is_correct() {
        let cfg = default_config();
        let g = cfg.effectiveness_matrix();

        // Increase CW motors (FR=0, RL=1), decrease CCW (FL=2, RR=3)
        let du = nalgebra::Vector4::new(1.0, 1.0, -1.0, -1.0);
        let virtual_control = g * du;
        assert!(virtual_control[2] > 0.0, "expected positive yaw, got {}", virtual_control[2]);
    }

    #[test]
    fn actuator_limits_are_ordered_and_positive() {
        let cfg = default_config();
        let (u_min, u_max) = cfg.actuator_limits();
        assert!(u_min >= 0.0);
        assert!(u_max > u_min);
        assert!((u_max - 3000.0_f32.powi(2)).abs() < 1.0);
    }

    #[test]
    fn motor_model_coefficients_match_linear_map() {
        let cfg = default_config();
        let (c0, c1) = cfg.motor_model_coefficients();
        assert_eq!(c0, 0.0);
        assert_eq!(c1, cfg.motor_omega_max);
    }
}
