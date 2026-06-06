//! Converts physics truth state into a noisy IMU reading in body frame.
use nalgebra::Vector3;
use rand::Rng;
use rand_distr::{Distribution, Normal};

use crate::physics::{VehicleState, GRAVITATIONAL_CONSTANT};
use ozonide_core::msgs::ImuData;

/// Additive white Gaussian noise parameters for the IMU.
pub struct ImuNoise {
    /// Accelerometer noise density (m/s²/√Hz). Applied per axis.
    pub accel_noise_std: f64,
    /// Accelerometer bias m/s². Constant per-axis, sampled from N(0, σ) at init
    pub accel_bias_std: f64,
    /// Accelerometer random walk m/s²/√s. Sample bias += N(0, σ_walk) × √dt each step
    pub accel_walk_std: f64,
    /// Gyroscope noise density (rad/s/√Hz). Applied per axis.
    pub gyro_noise_std: f64,
    /// Gyroscope bias rad/s
    pub gyro_bias_std: f64,
    /// Gyroscope random walk rad/s/√s. Sample bias += N(0, σ_walk) × √dt each step
    pub gyro_walk_std: f64,
}

impl Default for ImuNoise {
    fn default() -> Self {
        Self {
            accel_noise_std: 0.015,
            accel_bias_std: 0.05,
            accel_walk_std: 0.001,
            gyro_noise_std: 0.0011,
            gyro_bias_std: 0.005,
            gyro_walk_std: 0.0001,
        }
    }
}

/// Full IMU model: tracks world-frame velocity to derive acceleration between steps.
pub struct ImuModel {
    pub noise_parameters: ImuNoise,
    accel_bias_state: Vector3<f64>,
    gyro_bias_state: Vector3<f64>,
    last_velocity: Vector3<f64>,
    last_time_us: u64,
}

impl ImuModel {
    pub fn new(noise_parameters: ImuNoise, rng: &mut impl Rng) -> Self {
        let accel_bias_distribution = Normal::new(0.0, noise_parameters.accel_bias_std).unwrap();
        let accel_bias_state = Vector3::new(
            accel_bias_distribution.sample(rng),
            accel_bias_distribution.sample(rng),
            accel_bias_distribution.sample(rng),
        );
        let gyro_bias_distribution = Normal::new(0.0, noise_parameters.gyro_bias_std).unwrap();
        let gyro_bias_state = Vector3::new(
            gyro_bias_distribution.sample(rng),
            gyro_bias_distribution.sample(rng),
            gyro_bias_distribution.sample(rng),
        );
        Self {
            noise_parameters,
            accel_bias_state,
            gyro_bias_state,
            last_velocity: Vector3::zeros(),
            last_time_us: 0,
        }
    }

    pub fn measure(
        &mut self,
        state: &VehicleState,
        sim_time_us: u64,
        rng: &mut impl Rng,
    ) -> ImuData {
        let normal_distribution = Normal::new(0.0, 1.0).unwrap();
        let dt = u64::saturating_sub(sim_time_us, self.last_time_us) as f64 * 1e-6;
        let mut acceleration_world = Vector3::zeros();
        if dt > 0.0 {
            acceleration_world = (state.linear_velocity - self.last_velocity) / dt;
            let dt_sqrt = dt.sqrt();
            self.accel_bias_state += Vector3::new(
                normal_distribution.sample(rng),
                normal_distribution.sample(rng),
                normal_distribution.sample(rng),
            ) * self.noise_parameters.accel_walk_std
                * dt_sqrt;
            self.gyro_bias_state += Vector3::new(
                normal_distribution.sample(rng),
                normal_distribution.sample(rng),
                normal_distribution.sample(rng),
            ) * self.noise_parameters.gyro_walk_std
                * dt_sqrt;
        }
        self.last_velocity = state.linear_velocity;
        self.last_time_us = sim_time_us;

        // Specific force in world frame: acceleration minus gravity (ENU: gravity = -Z).
        let specific_force_world =
            acceleration_world + Vector3::new(0.0, 0.0, GRAVITATIONAL_CONSTANT);
        let specific_force_body = state
            .attitude
            .inverse_transform_vector(&specific_force_world);

        let acceleration = std::array::from_fn(|i| {
            (specific_force_body[i]
                + self.accel_bias_state[i]
                + self.noise_parameters.accel_noise_std * normal_distribution.sample(rng))
                as f32
        });

        let angular_velocity = std::array::from_fn(|i| {
            (state.angular_velocity[i]
                + self.gyro_bias_state[i]
                + self.noise_parameters.gyro_noise_std * normal_distribution.sample(rng))
                as f32
        });
        ImuData {
            timestamp_us: sim_time_us,
            linear_acceleration: acceleration,
            angular_velocity: angular_velocity,
            temperature: 25.0,
        }
    }
}
