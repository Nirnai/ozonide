//! Converts physics truth state into a noisy IMU reading in body frame.

use rand::{Rng, RngExt};
use nalgebra::Vector3;

use crate::physics::{VehicleState, GRAVITATIONAL_CONSTANT};
use ozonide_core::msgs::ImuData;

/// Additive white Gaussian noise parameters for the IMU.
pub struct ImuNoise {
    /// Accelerometer noise density (m/s²/√Hz). Applied per axis.
    pub accel_noise: f64,
    /// Gyroscope noise density (rad/s/√Hz). Applied per axis.
    pub gyro_noise: f64,
}

impl Default for ImuNoise {
    fn default() -> Self {
        Self { accel_noise: 0.003, gyro_noise: 0.0002 }
    }
}

/// Box-Muller transform: returns a standard normal sample using two uniform draws.
fn gaussian(rng: &mut impl Rng) -> f64 {
    use std::f64::consts::TAU;
    let u1: f64 = rng.random_range(1e-10..1.0);
    let u2: f64 = rng.random();
    (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
}

/// Full IMU model: tracks world-frame velocity to derive acceleration between steps.
pub struct ImuModel {
    pub noise: ImuNoise,
    last_velocity: Vector3<f64>,
    last_time_us: u64,
}

impl ImuModel {
    pub fn new(noise: ImuNoise) -> Self {
        Self { noise, last_velocity: Vector3::zeros(), last_time_us: 0 }
    }

    /// Produce an ImuData sample from the current physics state.
    ///
    /// Acceleration is estimated from the finite difference of world-frame velocity
    /// across consecutive calls, then transformed to body frame. Specific force equals
    /// body-frame acceleration minus gravity, matching what a real accelerometer
    /// reports (zero in free fall, +g·ẑ at rest on the bench).
    ///
    /// TODO (Step 3): Replace finite-difference acceleration with truth acceleration
    /// exposed directly from the RK4 `derivatives()` function. Finite differencing
    /// introduces one-step lag and amplifies noise at high sample rates.
    pub fn measure(&mut self, state: &VehicleState, sim_time_us: u64, rng: &mut impl Rng) -> ImuData {
        let dt = (sim_time_us.saturating_sub(self.last_time_us)) as f64 * 1e-6;
        let a_world = if dt > 0.0 {
            (state.linear_velocity - self.last_velocity) / dt
        } else {
            Vector3::zeros()
        };
        self.last_velocity = state.linear_velocity;
        self.last_time_us = sim_time_us;

        // Specific force in world frame: acceleration minus gravity (ENU: gravity = -Z).
        let sf_world = a_world + Vector3::new(0.0, 0.0, GRAVITATIONAL_CONSTANT);
        let sf_body = state.attitude.inverse_transform_vector(&sf_world);

        let acceleration = std::array::from_fn(|i| {
            (sf_body[i] + self.noise.accel_noise * gaussian(rng)) as f32
        });

        let angular_velocity = std::array::from_fn(|i| {
            (state.angular_velocity[i] + self.noise.gyro_noise * gaussian(rng)) as f32
        });
        ImuData {
            timestamp_us: sim_time_us,
            linear_acceleration: acceleration,
            angular_velocity: angular_velocity,
            temperature: 25.0,
        }
    }
}
