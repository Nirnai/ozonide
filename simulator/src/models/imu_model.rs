//! Converts physics truth state into a noisy IMU reading in body frame.
use nalgebra::Vector3;
use rand::Rng;
use rand_distr::{Distribution, Normal};

use crate::physics::{VehicleState, VehicleStateDot, GRAVITATIONAL_CONSTANT};
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

pub struct ImuModel {
    pub noise_parameters: ImuNoise,
    /// Toggle instantaneous Gaussian noise on each measurement.
    pub white_noise_enabled: bool,
    /// Toggle the constant per-axis bias offset (sampled once at construction).
    pub bias_enabled: bool,
    /// Toggle random walk — the slow drift that accumulates over time.
    pub walk_enabled: bool,
    accel_bias: Vector3<f64>,
    accel_walk: Vector3<f64>,
    gyro_bias: Vector3<f64>,
    gyro_walk: Vector3<f64>,
    last_time_us: u64,
}

impl ImuModel {
    pub fn new(noise_parameters: ImuNoise, rng: &mut impl Rng) -> Self {
        let accel_dist = Normal::new(0.0, noise_parameters.accel_bias_std).unwrap();
        let accel_bias = Vector3::new(
            accel_dist.sample(rng),
            accel_dist.sample(rng),
            accel_dist.sample(rng),
        );
        let gyro_dist = Normal::new(0.0, noise_parameters.gyro_bias_std).unwrap();
        let gyro_bias = Vector3::new(
            gyro_dist.sample(rng),
            gyro_dist.sample(rng),
            gyro_dist.sample(rng),
        );
        Self {
            noise_parameters,
            white_noise_enabled: true,
            bias_enabled: true,
            walk_enabled: true,
            accel_bias,
            accel_walk: Vector3::zeros(),
            gyro_bias,
            gyro_walk: Vector3::zeros(),
            last_time_us: 0,
        }
    }

    pub fn measure(
        &mut self,
        state: &VehicleState,
        state_dot: &VehicleStateDot,
        sim_time_us: u64,
        rng: &mut impl Rng,
    ) -> ImuData {
        let dt = u64::saturating_sub(sim_time_us, self.last_time_us) as f64 * 1e-6;
        self.last_time_us = sim_time_us;

        if self.walk_enabled && dt > 0.0 {
            let n = Normal::new(0.0, 1.0).unwrap();
            let s = dt.sqrt();
            self.accel_walk += Vector3::new(n.sample(rng), n.sample(rng), n.sample(rng))
                * self.noise_parameters.accel_walk_std * s;
            self.gyro_walk += Vector3::new(n.sample(rng), n.sample(rng), n.sample(rng))
                * self.noise_parameters.gyro_walk_std * s;
        }

        // Specific force in world frame: acceleration minus gravity (ENU: gravity = -Z).
        let specific_force_world =
            state_dot.linear_acceleration + Vector3::new(0.0, 0.0, GRAVITATIONAL_CONSTANT);
        let specific_force_body = state
            .attitude
            .inverse_transform_vector(&specific_force_world);

        let accel_bias = if self.bias_enabled { self.accel_bias } else { Vector3::zeros() };
        let accel_walk = if self.walk_enabled { self.accel_walk } else { Vector3::zeros() };
        let gyro_bias  = if self.bias_enabled { self.gyro_bias  } else { Vector3::zeros() };
        let gyro_walk  = if self.walk_enabled { self.gyro_walk  } else { Vector3::zeros() };

        let n = Normal::new(0.0, 1.0).unwrap();
        let accel_white = if self.white_noise_enabled { self.noise_parameters.accel_noise_std } else { 0.0 };
        let gyro_white  = if self.white_noise_enabled { self.noise_parameters.gyro_noise_std  } else { 0.0 };

        let linear_acceleration = std::array::from_fn(|i| {
            (specific_force_body[i] + accel_bias[i] + accel_walk[i]
                + accel_white * n.sample(rng)) as f32
        });
        let angular_velocity = std::array::from_fn(|i| {
            (state.angular_velocity[i] + gyro_bias[i] + gyro_walk[i]
                + gyro_white * n.sample(rng)) as f32
        });

        ImuData { timestamp_us: sim_time_us, linear_acceleration, angular_velocity, temperature: 25.0 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::VehicleState;
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    fn noise_only() -> ImuNoise {
        ImuNoise {
            accel_bias_std: 0.0,
            accel_walk_std: 0.0,
            gyro_bias_std: 0.0,
            gyro_walk_std: 0.0,
            ..ImuNoise::default()
        }
    }

    fn bias_only() -> ImuNoise {
        ImuNoise {
            accel_noise_std: 0.0,
            accel_walk_std: 0.0,
            gyro_noise_std: 0.0,
            gyro_walk_std: 0.0,
            ..ImuNoise::default()
        }
    }

    fn walk_only() -> ImuNoise {
        ImuNoise {
            accel_noise_std: 0.0,
            accel_bias_std: 0.0,
            gyro_noise_std: 0.0,
            gyro_bias_std: 0.0,
            ..ImuNoise::default()
        }
    }

    fn collect_samples(n: usize, noise_parameters: ImuNoise, seed: u64) -> Vec<ImuData> {
        let state = VehicleState::default();
        let state_dot = VehicleStateDot::default();
        let sampling_dt_us = 1000;
        let mut rng = StdRng::seed_from_u64(seed);
        let mut model = ImuModel::new(noise_parameters, &mut rng);
        let mut samples = Vec::new();
        for i in 0..n {
            // Skip zero case for proper statistics
            let sim_time_us = ((i + 1) * sampling_dt_us) as u64;
            let sample = model.measure(&state, &state_dot, sim_time_us, &mut rng);
            samples.push(sample);
        }
        samples
    }

    fn estimate_gaussian_parameters(samples: Vec<f32>) -> (f32, f32) {
        let n = samples.len();
        let mean = samples.iter().sum::<f32>() / n as f32;
        let var = samples.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / n as f32;
        let std = var.sqrt();
        (mean, std)
    }

    #[test]
    fn first_call_with_zero_dt_produces_valid_output() {
        let mut rng = StdRng::seed_from_u64(0);
        let mut model = ImuModel::new(ImuNoise::default(), &mut rng);
        let state = VehicleState::default();
        let state_dot = VehicleStateDot::default();
        let sample = model.measure(&state, &state_dot, 0, &mut rng); // sim_time_us == last_time_us == 0 → dt = 0
        assert!(sample.linear_acceleration.iter().all(|v| v.is_finite()));
        assert!(sample.angular_velocity.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn white_noise_std_dev_and_mean_match_parameters() {
        let noise_parameters = ImuNoise::default();
        let seed = 42;
        let num_samples = 10000;
        let mean_tolerance = 0.005;
        let samples = collect_samples(num_samples, noise_only(), seed);
        for axis in 0..3 {
            let values: Vec<f32> = samples
                .iter()
                .map(|s| s.linear_acceleration[axis])
                .collect();
            let (mean, std) = estimate_gaussian_parameters(values);
            let expected_mean = if axis == 2 { 9.806 } else { 0.0 };
            assert!(
                (mean - expected_mean).abs() < mean_tolerance,
                "accel[{axis}] mean={mean}"
            );
            assert!(
                (std - noise_parameters.accel_noise_std as f32).abs() < 0.015 * 0.10,
                "accel[{axis}] std_dev={std}"
            );
        }
    }

    #[test]
    fn bias_is_constant_and_drawn_from_correct_distribution() {
        // Part 1: with no noise and no walk, every sample must equal the first.
        let samples = collect_samples(100, bias_only(), 42);
        for axis in 0..3 {
            let reference = samples[0].linear_acceleration[axis];
            for s in &samples {
                assert_eq!(
                    s.linear_acceleration[axis], reference,
                    "accel[{axis}] changed between samples — bias is not constant"
                );
            }
        }

        // Part 2: across many model instances the bias values should be drawn
        // from N(0, accel_bias_std). At rest with identity attitude,
        // axis-X output = 0 + bias_x (no noise, no horizontal specific force).
        let num_instances = 200;
        let mut accel_biases: Vec<f32> = Vec::new();
        let mut gyro_biases: Vec<f32> = Vec::new();
        for seed in 0..num_instances as u64 {
            let s = collect_samples(1, bias_only(), seed);
            accel_biases.push(s[0].linear_acceleration[0]);
            gyro_biases.push(s[0].angular_velocity[0]);
        }
        let (_, accel_std) = estimate_gaussian_parameters(accel_biases);
        let (_, gyro_std) = estimate_gaussian_parameters(gyro_biases);

        let target_accel = ImuNoise::default().accel_bias_std as f32;
        let target_gyro = ImuNoise::default().gyro_bias_std as f32;
        assert!(
            (accel_std - target_accel).abs() < target_accel * 0.20,
            "accel bias std={accel_std}, expected≈{target_accel}"
        );
        assert!(
            (gyro_std - target_gyro).abs() < target_gyro * 0.20,
            "gyro bias std={gyro_std}, expected≈{target_gyro}"
        );
    }

    #[test]
    fn random_walk_variance_grows_linearly_with_time() {
        // With walk_only: initial bias = 0, no white noise.
        // accel[0] output = 0 + accumulated_walk_x.
        // Walk variance after t seconds = σ_walk² × t, so var(10s)/var(1s) ≈ 10.
        let num_instances = 200;
        let mut vals_1s: Vec<f32> = Vec::new();
        let mut vals_10s: Vec<f32> = Vec::new();
        for seed in 0..num_instances as u64 {
            let samples = collect_samples(10_000, walk_only(), seed);
            vals_1s.push(samples[999].linear_acceleration[0]);    // index 999 → t = 1 s
            vals_10s.push(samples[9_999].linear_acceleration[0]); // index 9999 → t = 10 s
        }
        let variance_ratio = {
            let (_, std_1s) = estimate_gaussian_parameters(vals_1s);
            let (_, std_10s) = estimate_gaussian_parameters(vals_10s);
            (std_10s / std_1s).powi(2)
        };
        assert!(
            (variance_ratio - 10.0).abs() < 10.0 * 0.30,
            "variance ratio={variance_ratio}, expected≈10"
        );
    }
}
