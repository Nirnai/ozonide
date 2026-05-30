//! Converts physics truth state into a noisy IMU reading in body frame.

use rand::Rng;

use crate::physics::{world_to_body, State};
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
    let u1: f64 = rng.gen_range(1e-10..1.0);
    let u2: f64 = rng.gen();
    (-2.0 * u1.ln()).sqrt() * (TAU * u2).cos()
}

/// Full IMU model: tracks world-frame velocity to derive acceleration between steps.
pub struct ImuModel {
    pub noise: ImuNoise,
    last_vel: [f64; 3],
    last_time_us: u64,
}

impl ImuModel {
    pub fn new(noise: ImuNoise) -> Self {
        Self { noise, last_vel: [0.0; 3], last_time_us: 0 }
    }

    /// Produce an ImuData sample from the current physics state.
    ///
    /// Acceleration is estimated from the finite difference of `state.vel` across
    /// consecutive calls, then transformed to body frame. Specific force equals
    /// body-frame acceleration minus the body-frame gravity vector, matching what
    /// a real accelerometer reports (zero in free fall, +g·ẑ on the bench).
    pub fn measure(&mut self, state: &State, sim_time_us: u64, rng: &mut impl Rng) -> ImuData {
        const G: f64 = 9.80665;

        let dt = (sim_time_us.saturating_sub(self.last_time_us)) as f64 * 1e-6;
        let a_world = if dt > 0.0 {
            [
                (state.vel[0] - self.last_vel[0]) / dt,
                (state.vel[1] - self.last_vel[1]) / dt,
                (state.vel[2] - self.last_vel[2]) / dt,
            ]
        } else {
            [0.0; 3]
        };
        self.last_vel = state.vel;
        self.last_time_us = sim_time_us;

        // Specific force in world frame: acceleration minus gravity (ENU: gravity = -Z).
        let sf_world = [a_world[0], a_world[1], a_world[2] + G];
        let sf_body = world_to_body(state.quat, sf_world);

        let accel = sf_body.map(|v| (v + self.noise.accel_noise * gaussian(rng)) as f32);
        let angular_velocity =
            state.omega.map(|v| (v + self.noise.gyro_noise * gaussian(rng)) as f32);

        ImuData {
            timestamp_us: sim_time_us,
            linear_acceleration: accel,
            angular_velocity,
            temperature: 25.0,
        }
    }
}
