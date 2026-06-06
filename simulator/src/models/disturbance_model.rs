//! External disturbance forces and torques applied to the quadrotor.
//!
//! Three disturbance modes are available and can be switched at runtime:
//!
//! - [`DisturbanceType::Gaussian`] — independent white-noise forces each step,
//!   modelling high-frequency uncorrelated turbulence
//! - [`DisturbanceType::OrnsteinUhlenbeck`] — mean-reverting correlated gusts
//!   with characteristic time `τ_wind` from [`VehicleParameters`]
//! - [`DisturbanceType::NoDisturbance`] — zero forces; produces a deterministic
//!   simulation for baseline testing

use nalgebra::Vector3;
use rand::Rng;
use rand_distr::{Distribution, Normal};

use crate::physics::VehicleParameters;

/// Selects the statistical model used to generate external disturbances.
#[derive(Copy, Clone, PartialEq)]
pub enum DisturbanceType {
    /// Independent Gaussian noise applied each physics step.
    /// Each call to [`DisturbanceModel::generate`] draws a fresh independent sample.
    Gaussian,
    /// Ornstein-Uhlenbeck mean-reverting process.
    ///
    /// Models realistic wind gusts with temporal correlation:
    /// ```text
    /// dF/dt = −F / τ_wind + σ · w(t)
    /// ```
    /// The state (`force`, `torque`) persists between steps and decays toward zero
    /// with time constant `τ_wind`.
    OrnsteinUhlenbeck,
    /// No external disturbances. Used for deterministic baseline runs or when
    /// only an initial-condition perturbation (applied at reset) is desired.
    NoDisturbance,
}

/// Runtime-configurable source of external aerodynamic disturbances.
///
/// Call [`generate`](DisturbanceModel::generate) exactly once per physics step.
/// The returned `(force, torque)` tuple should be passed into the `derivative`
/// function unchanged across all four RK4 stages.
pub struct DisturbanceModel {
    /// Active disturbance model. Safe to change between physics steps.
    pub disturbance_type: DisturbanceType,
    /// Accumulated disturbance force in the ENU world frame (N).
    /// Retained between steps for the OU process; reset to zero on mode change.
    pub force: Vector3<f64>,
    /// Accumulated disturbance torque in the body frame (N·m).
    /// Retained between steps for the OU process; reset to zero on mode change.
    pub torque: Vector3<f64>,
}

impl DisturbanceModel {

    /// Creates a new model with zero initial force and torque.
    pub fn new(disturbance_type: DisturbanceType) -> Self {
        Self { disturbance_type, force: Vector3::zeros(), torque: Vector3::zeros() }
    }

    /// Samples the next disturbance wrench and advances internal state.
    ///
    /// Returns `(force_world, torque_body)` to be applied for this physics step.
    /// Must be called exactly once per step; calling multiple times per step would
    /// advance the OU process too fast.
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
                // Euler-Maruyama discretisation of the OU SDE:
                //   dF = −F/τ · dt + σ · dW,   dW ~ N(0, dt)
                self.force =
                    self.force + (-self.force / params.tau_wind) * dt + force_sample * dt.sqrt();
                self.torque =
                    self.torque + (-self.torque / params.tau_wind) * dt + torque_sample * dt.sqrt();
                (self.force, self.torque)
            },
            DisturbanceType::NoDisturbance => (Vector3::zeros(), Vector3::zeros()),
        }
    }

    /// Draws independent Gaussian samples for force and torque.
    ///
    /// Magnitudes are scaled to vehicle weight:
    /// `σ_F = 0.027 · m · g ≈ 0.13 N` (at m = 0.5 kg).
    fn sample_random_disturbances(
        &self,
        rng: &mut impl Rng,
        vehicle_parameters: &VehicleParameters,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let force_dist = Normal::new(0.0, vehicle_parameters.sigma_wind_force).unwrap();
        let force_sample = Vector3::new(
            force_dist.sample(rng),
            force_dist.sample(rng),
            force_dist.sample(rng),
        );
        let torque_dist = Normal::new(0.0, vehicle_parameters.sigma_wind_torque).unwrap();
        let torque_sample = Vector3::new(
            torque_dist.sample(rng),
            torque_dist.sample(rng),
            torque_dist.sample(rng),
        );
        (force_sample, torque_sample)
    }
}
