use crate::msgs::{BatteryStatus, ImuData, ActuatorTelemetry, VehicleState};

/// A short-lived bundle of sensor readings passed to [`StateEstimator::update`]
/// on each estimation cycle.
///
/// Each field is `Option` because not all sensors are available in every
/// configuration — an optical-flow-only setup has no IMU, a minimal hover
/// controller may have no barometer or GPS. The estimator is responsible for
/// deciding which fields it requires and how to handle absent measurements.
///
/// `SensorData` is constructed in the state estimation task each iteration and
/// is never serialized or put on a topic. The lifetime `'a` ties the references
/// to the sensor readings that were read from their respective topics.
pub struct SensorData<'a> {
    /// Latest IMU sample. `None` if IMU is not available in this configuration.
    pub imu: Option<&'a ImuData>,
    /// Latest actuator telemetry (eRPM-derived Ω, rad/s). `None` until first
    /// frame or on dropout — maps to `VehicleState::motor_speed_valid`.
    pub actuator: Option<&'a ActuatorTelemetry>,
    /// Latest battery measurement. Slowly varying; staleness is harmless.
    pub battery: Option<&'a BatteryStatus>,
}

/// Abstraction over any state estimation algorithm.
///
/// Implementors receive sensor readings via [`update`](StateEstimator::update)
/// and return a fully-populated [`VehicleState`]. Fields the estimator cannot
/// fill (e.g. position without GPS) should be left at their [`Default`] value.
///
/// # Implementing
///
/// - `update` is called at the rate of the fastest sensor (typically IMU at 1 kHz).
///   Slower sensors arrive as `Some` only when a new measurement is available;
///   the estimator runs its prediction step every call and its correction step
///   only when the relevant field is `Some`.
/// - `reset` is called on arming or mode transitions to clear accumulated state
///   such as integrators and covariance matrices.
pub trait StateEstimator {
    /// Advances the estimator by one step and returns the current state estimate.
    fn update(&mut self, sensor_data: &SensorData<'_>) -> VehicleState;
    /// Resets all internal state to initial conditions.
    fn reset(&mut self);
}