use nalgebra::Vector3;
use rand::Rng;
use rand_distr::{Distribution, Normal};

use crate::physics::VehicleParameters;

#[derive(Copy, Clone, PartialEq)]
pub enum DisturbanceType {
    Gaussian,
    OrnsteinUhlenbeck,
    NoDisturbance,
}
pub struct DisturbanceModel {
    pub disturbance_type: DisturbanceType,
    pub force: Vector3<f64>,
    pub torque: Vector3<f64>,
}

impl DisturbanceModel {

    pub fn new(
        disturbance_type: DisturbanceType
    ) -> Self {
        Self { disturbance_type, force: Vector3::zeros(), torque: Vector3::zeros() }
    }

    pub fn generate(
        &mut self,
        params: &VehicleParameters,
        rng: &mut impl Rng,
        dt: f64,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let (force_sample, torque_sample) = self.sample_random_disturbances(rng, params);

        match self.disturbance_type {
            DisturbanceType::Gaussian => (force_sample, torque_sample),
            DisturbanceType::OrnsteinUhlenbeck => {
                // Ornstein-Uhlenbeck process to model gusts:
                //       dF/dt = -F / τ_wind + σ · w(t)
                //       F[k+1] = F[k] + (-F[k] / τ_wind + σ · N(0,1)) · dt
                self.force =
                    self.force + (-self.force / params.tau_wind) * dt + force_sample * dt.sqrt();
                self.torque = self.torque + (-self.torque / params.tau_wind) * dt + torque_sample * dt.sqrt();
                (self.force, self.torque)
            },
            DisturbanceType::NoDisturbance => (Vector3::zeros(), Vector3::zeros())
        }
    }

    fn sample_random_disturbances(
        &self,
        rng: &mut impl Rng,
        vehicle_parameters: &VehicleParameters,
    ) -> (Vector3<f64>, Vector3<f64>) {
        // F_dist ~ N(0, σ_F)  per axis,   σ_F = 0.027 · m · g  ≈ 0.13 N  (at m = 0.5 kg)
        let force_distribution = Normal::new(0.0, vehicle_parameters.sigma_wind_force).unwrap();
        let force_sample = Vector3::new(
            force_distribution.sample(rng),
            force_distribution.sample(rng),
            force_distribution.sample(rng),
        );
        let torque_distribution = Normal::new(0.0, vehicle_parameters.sigma_wind_torque).unwrap();
        let torque_sample = Vector3::new(
            torque_distribution.sample(rng),
            torque_distribution.sample(rng),
            torque_distribution.sample(rng),
        );
        (force_sample, torque_sample)
    }
}


