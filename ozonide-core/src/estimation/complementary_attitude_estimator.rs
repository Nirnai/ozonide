//! Complementary filter for roll and pitch attitude estimation from IMU data.
//!
//! ## Why a complementary filter?
//!
//! Two IMU sensors give conflicting information about attitude:
//!
//! - **Gyroscope**: measures angular rate directly. Integrating it gives a smooth,
//!   low-latency angle estimate — but integration accumulates drift over time.
//! - **Accelerometer**: measures specific force. When the vehicle is not accelerating,
//!   it measures gravity, whose direction gives an absolute roll/pitch reference —
//!   but it is noisy and corrupted by any linear acceleration of the vehicle.
//!
//! The complementary filter exploits the fact that these two error sources occupy
//! different frequency bands: gyro drift is a low-frequency phenomenon, accelerometer
//! noise is high-frequency. By high-pass filtering the gyro and low-pass filtering the
//! accelerometer and summing them, you get a single estimate that is accurate at all
//! frequencies:
//!
//! ```text
//! angle[k] = α · (angle[k-1] + gyro · dt) + (1 − α) · accel_angle
//! ```
//!
//! `α` controls the crossover frequency. At `α = 0.98` the accelerometer correction
//! has a time constant of roughly `dt / (1 − α)` seconds (50 ms at 1 kHz), meaning
//! slow gyro drift is corrected while fast accelerometer noise is rejected.
//!
//! ## Tilt from the accelerometer
//!
//! When linear acceleration is small the specific force vector points opposite to
//! gravity. Roll and pitch can then be read directly from its direction:
//!
//! ```text
//! roll_accel  = atan2(ay,  az)
//! pitch_accel = atan2(−ax, sqrt(ay² + az²))
//! ```
//!
//! These angles are only reliable near hover. Aggressive manoeuvres corrupt the
//! accelerometer reference, but the gyro integration carries the estimate through
//! short transients and `α` limits how quickly the corrupted accel reference is
//! trusted.
//!
//! ## Yaw
//!
//! Gravity carries no information about yaw (rotation about the vertical axis), so
//! yaw cannot be corrected by the accelerometer. The filter integrates yaw from the
//! gyroscope and accepts slow drift. Absolute yaw correction requires a magnetometer
//! or external reference (GPS, VIO), neither of which is available at this stage.

use libm::{atan2f, sqrtf};
use nalgebra::UnitQuaternion;

use crate::{
    msgs::{StateValidity, VehicleState},
    traits::{SensorData, StateEstimator},
};

/// Complementary filter implementing [`StateEstimator`].
///
/// Call [`StateEstimator::update`] once per IMU sample to advance the estimate.
/// On the first call the filter records the initial timestamp and returns a
/// level default state; meaningful output begins from the second call.
pub struct ComplementaryAttitudeEstimator {
    /// Timestamp of the previous IMU sample (µs). `None` until the first call.
    last_timestamp_us: Option<u64>,
    /// Blending coefficient in `(0, 1)`. Fraction of the estimate drawn from gyro
    /// integration. The accelerometer contributes `1 − α`. Typical value: `0.98`.
    alpha: f32,
    /// Current roll estimate (rad).
    roll: f32,
    /// Current pitch estimate (rad).
    pitch: f32,
    /// Current yaw estimate (rad). Integrated from gyro only — drifts over time.
    yaw: f32,
}

impl ComplementaryAttitudeEstimator {
    /// Creates a new filter initialised to level attitude (roll = pitch = yaw = 0).
    ///
    /// `alpha` must be in `(0, 1)`. Values close to 1 (e.g. `0.98`) trust the
    /// gyro heavily and correct drift slowly; lower values correct faster but
    /// let accelerometer noise into the estimate.
    pub fn new(alpha: f32) -> Self {
        Self { last_timestamp_us: None, alpha, roll: 0.0, pitch: 0.0, yaw: 0.0 }
    }
}

impl StateEstimator for ComplementaryAttitudeEstimator {
    /// Advances the estimate by one step.
    ///
    /// Requires `sensor_data.imu` to be `Some` — returns a default (level) state
    /// if IMU data is absent. On the first call with valid IMU data the filter
    /// records the initial timestamp and returns a default state; meaningful output
    /// begins from the second call.
    fn update(&mut self, sensor_data: &SensorData<'_>) -> VehicleState {
        let Some(imu) = sensor_data.imu else {
            return VehicleState::default();
        };

        let Some(last_timestamp_us) = self.last_timestamp_us else {
            self.last_timestamp_us = Some(imu.timestamp_us);
            return VehicleState::default();
        };

        // Simulator reset: timestamp went backward — clear all state and reseed.
        if imu.timestamp_us < last_timestamp_us {
            self.reset();
            self.last_timestamp_us = Some(imu.timestamp_us);
            return VehicleState::default();
        }

        let dt = (imu.timestamp_us - last_timestamp_us) as f32 * 1e-6;
        self.last_timestamp_us = Some(imu.timestamp_us);

        let gyro_weight = self.alpha;
        let accel_weight = 1.0 - self.alpha;
        let [a_x, a_y, a_z] = imu.specific_force;
        let accel_roll = atan2f(a_y, a_z);
        let accel_pitch = atan2f(-a_x, sqrtf(a_y * a_y + a_z * a_z));
        let [gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate] = imu.angular_velocity;

        self.roll  = gyro_weight * (self.roll  + gyro_roll_rate  * dt) + accel_weight * accel_roll;
        self.pitch = gyro_weight * (self.pitch + gyro_pitch_rate * dt) + accel_weight * accel_pitch;
        self.yaw  += gyro_yaw_rate * dt; // pure gyro integration, no correction

        let q = UnitQuaternion::from_euler_angles(self.roll, self.pitch, self.yaw);
        let c = q.coords; // nalgebra stores [x, y, z, w]

        VehicleState {
            timestamp_us: imu.timestamp_us,
            valid: StateValidity::ATTITUDE,
            attitude: [c.x, c.y, c.z, c.w],
            angular_velocity: imu.angular_velocity,
            specific_force: imu.specific_force,
            ..VehicleState::default()
        }
    }

    /// Resets all filter state to level attitude (roll = pitch = yaw = 0).
    fn reset(&mut self) {
        self.last_timestamp_us = None;
        self.roll  = 0.0;
        self.pitch = 0.0;
        self.yaw   = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::ImuData;
    use nalgebra::{Quaternion, UnitQuaternion};

    const G: f32 = 9.81;
    const EPS: f32 = 1e-5;

    fn imu_at(timestamp_us: u64, accel: [f32; 3], gyro: [f32; 3]) -> ImuData {
        ImuData { timestamp_us, specific_force: accel, angular_velocity: gyro, temperature: 25.0 }
    }

    fn level_imu(timestamp_us: u64) -> ImuData {
        imu_at(timestamp_us, [0.0, 0.0, G], [0.0, 0.0, 0.0])
    }

    fn assert_approx(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < EPS, "got {actual}, expected {expected}");
    }

    /// Extract (roll, pitch, yaw) Euler angles from a VehicleState quaternion.
    fn euler(state: &VehicleState) -> (f32, f32, f32) {
        let [x, y, z, w] = state.attitude;
        let q = UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));
        q.euler_angles()
    }

    fn sensor(imu: &ImuData) -> SensorData<'_> {
        SensorData { imu: Some(imu), motor: None, battery: None }
    }

    #[test]
    fn first_call_returns_default_state() {
        let mut filter = ComplementaryAttitudeEstimator::new(0.98);
        let state = filter.update(&sensor(&level_imu(0)));
        let (roll, pitch, _) = euler(&state);
        assert_approx(roll, 0.0);
        assert_approx(pitch, 0.0);
        assert_approx(state.angular_velocity[2], 0.0);
    }

    #[test]
    fn level_accel_produces_zero_roll_and_pitch() {
        // alpha=0.0: pure accelerometer. Level specific force → roll=pitch=0.
        let mut filter = ComplementaryAttitudeEstimator::new(0.0);
        filter.update(&sensor(&level_imu(0)));
        let state = filter.update(&sensor(&level_imu(1000)));
        let (roll, pitch, _) = euler(&state);
        assert_approx(roll, 0.0);
        assert_approx(pitch, 0.0);
    }

    #[test]
    fn pure_accel_extracts_correct_roll_angle() {
        // alpha=0.0: output is entirely from accelerometer.
        // Tilt 30° right (right side down): a_y = G*sin(30°), a_z = G*cos(30°).
        let angle = 30_f32.to_radians();
        let accel = [0.0, G * angle.sin(), G * angle.cos()];
        let mut filter = ComplementaryAttitudeEstimator::new(0.0);
        filter.update(&sensor(&imu_at(0, accel, [0.0; 3])));
        let state = filter.update(&sensor(&imu_at(1000, accel, [0.0; 3])));
        let (roll, pitch, _) = euler(&state);
        assert_approx(roll, angle);
        assert_approx(pitch, 0.0);
    }

    #[test]
    fn pure_accel_extracts_correct_pitch_angle() {
        // alpha=0.0: nose-down 20°: a_x = -G*sin(20°), remaining force in a_z.
        let angle = 20_f32.to_radians();
        let accel = [-G * angle.sin(), 0.0, G * angle.cos()];
        let mut filter = ComplementaryAttitudeEstimator::new(0.0);
        filter.update(&sensor(&imu_at(0, accel, [0.0; 3])));
        let state = filter.update(&sensor(&imu_at(1000, accel, [0.0; 3])));
        let (_, pitch, _) = euler(&state);
        assert_approx(pitch, angle);
        let (roll, _, _) = euler(&state);
        assert_approx(roll, 0.0);
    }

    #[test]
    fn pure_gyro_integrates_roll_rate() {
        // alpha=1.0: output is entirely from gyro integration.
        // 1 rad/s roll rate for 1 ms → 0.001 rad.
        let rate = 1.0_f32;
        let dt_us = 1000_u64;
        let expected_roll = rate * (dt_us as f32 * 1e-6);
        let mut filter = ComplementaryAttitudeEstimator::new(1.0);
        filter.update(&sensor(&imu_at(0, [0.0, 0.0, G], [0.0; 3])));
        let state = filter.update(&sensor(&imu_at(dt_us, [0.0, 0.0, G], [rate, 0.0, 0.0])));
        let (roll, _, _) = euler(&state);
        assert_approx(roll, expected_roll);
    }

    #[test]
    fn gyro_rates_are_passed_through_directly() {
        let gyro = [0.1, -0.2, 0.3];
        let mut filter = ComplementaryAttitudeEstimator::new(0.98);
        filter.update(&sensor(&imu_at(0, [0.0, 0.0, G], [0.0; 3])));
        let state = filter.update(&sensor(&imu_at(1000, [0.0, 0.0, G], gyro)));
        assert_approx(state.angular_velocity[0], gyro[0]);
        assert_approx(state.angular_velocity[1], gyro[1]);
        assert_approx(state.angular_velocity[2], gyro[2]);
    }

    #[test]
    fn reset_returns_to_level_attitude() {
        let mut filter = ComplementaryAttitudeEstimator::new(1.0);
        filter.update(&sensor(&imu_at(0, [0.0, 0.0, G], [0.0; 3])));
        // Spin up some roll via gyro integration.
        filter.update(&sensor(&imu_at(1000, [0.0, 0.0, G], [10.0, 0.0, 0.0])));
        filter.reset();
        // After reset the first call re-initialises the timestamp.
        filter.update(&sensor(&imu_at(2000, [0.0, 0.0, G], [0.0; 3])));
        let state = filter.update(&sensor(&imu_at(3000, [0.0, 0.0, G], [0.0; 3])));
        let (roll, pitch, _) = euler(&state);
        assert_approx(roll, 0.0);
        assert_approx(pitch, 0.0);
    }
}
