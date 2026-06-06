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
//! gyroscope and accepts slow drift. Yaw *rate* (directly from the gyro) is passed
//! to the yaw rate PID without integration, so control is unaffected by the drift.
//! Absolute yaw correction requires a magnetometer or external reference (GPS, VIO),
//! neither of which is available at this stage.

use libm::{atan2f, sqrtf};

use crate::msgs::ImuData;

/// Roll and pitch estimate produced by the complementary filter each IMU cycle.
#[derive(Default)]
pub struct AttitudeEstimate {
    /// Estimated roll angle (rad). Positive = right side down.
    pub roll: f32,
    /// Estimated pitch angle (rad). Positive = nose down.
    pub pitch: f32,
    /// Roll rate from gyroscope (rad/s).
    pub roll_rate: f32,
    /// Pitch rate from gyroscope (rad/s).
    pub pitch_rate: f32,
    /// Yaw rate from gyroscope (rad/s). Positive = CCW viewed from above.
    pub yaw_rate: f32,
}

/// Complementary filter state. Call [`update`](ComplementaryFilter::update) once
/// per IMU sample to advance the estimate.
pub struct ComplementaryFilter {
    /// Timestamp of the previous IMU sample (µs). `None` until the first call.
    last_timestamp_us: Option<u64>,
    /// Blending coefficient in `(0, 1)`. Fraction of the estimate drawn from gyro
    /// integration. The accelerometer contributes `1 − α`. Typical value: `0.98`.
    alpha: f32,
    /// Current roll estimate (rad).
    roll: f32,
    /// Current pitch estimate (rad).
    pitch: f32,
}

impl ComplementaryFilter {
    /// Creates a new filter initialised to level attitude (roll = pitch = 0).
    ///
    /// `alpha` must be in `(0, 1)`. Values close to 1 (e.g. `0.98`) trust the
    /// gyro heavily and correct drift slowly; lower values correct faster but
    /// let accelerometer noise into the estimate.
    pub fn new(alpha: f32) -> Self {
        Self { last_timestamp_us: None, alpha, roll: 0.0, pitch: 0.0 }
    }

    /// Advances the estimate by one IMU sample.
    ///
    /// `dt` is derived from `imu.timestamp_us` relative to the previous call.
    /// On the first call the filter initialises its timestamp and returns a
    /// level default estimate; meaningful output begins from the second call.
    pub fn update(&mut self, imu: &ImuData) -> AttitudeEstimate {
        let Some(last_timestamp_us) = self.last_timestamp_us else {
            self.last_timestamp_us = Some(imu.timestamp_us);
            return AttitudeEstimate::default();
        };
        let dt = (imu.timestamp_us - last_timestamp_us) as f32 * 1e-6;
        let gyro_weight = self.alpha;
        let accel_weight = 1.0 - self.alpha;
        let [a_x, a_y, a_z] = imu.linear_acceleration;
        let accel_roll = atan2f(a_y, a_z);
        let accel_pitch = atan2f(-a_x, sqrtf(a_y * a_y + a_z * a_z));
        let [gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate] = imu.angular_velocity;
        self.last_timestamp_us = Some(imu.timestamp_us);
        self.roll = gyro_weight * (self.roll + gyro_roll_rate * dt) + accel_weight * accel_roll;
        self.pitch = gyro_weight * (self.pitch + gyro_pitch_rate * dt) + accel_weight * accel_pitch;
        AttitudeEstimate {
            roll: self.roll,
            pitch: self.pitch,
            roll_rate: gyro_roll_rate,
            pitch_rate: gyro_pitch_rate,
            yaw_rate: gyro_yaw_rate,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const G: f32 = 9.81;
    const EPS: f32 = 1e-5;

    fn imu_at(timestamp_us: u64, accel: [f32; 3], gyro: [f32; 3]) -> ImuData {
        ImuData { timestamp_us, linear_acceleration: accel, angular_velocity: gyro, temperature: 25.0 }
    }

    fn level_imu(timestamp_us: u64) -> ImuData {
        imu_at(timestamp_us, [0.0, 0.0, G], [0.0, 0.0, 0.0])
    }

    fn assert_approx(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < EPS, "got {actual}, expected {expected}");
    }

    #[test]
    fn first_call_returns_default_estimate() {
        let mut filter = ComplementaryFilter::new(0.98);
        let estimate = filter.update(&level_imu(0));
        assert_approx(estimate.roll, 0.0);
        assert_approx(estimate.pitch, 0.0);
        assert_approx(estimate.yaw_rate, 0.0);
    }

    #[test]
    fn level_accel_produces_zero_roll_and_pitch() {
        // alpha=0.0: pure accelerometer. Level specific force → roll=pitch=0.
        let mut filter = ComplementaryFilter::new(0.0);
        filter.update(&level_imu(0));
        let estimate = filter.update(&level_imu(1000));
        assert_approx(estimate.roll, 0.0);
        assert_approx(estimate.pitch, 0.0);
    }

    #[test]
    fn pure_accel_extracts_correct_roll_angle() {
        // alpha=0.0: output is entirely from accelerometer.
        // Tilt 30° right (right side down): a_y = G*sin(30°), a_z = G*cos(30°).
        let angle = 30_f32.to_radians();
        let accel = [0.0, G * angle.sin(), G * angle.cos()];
        let mut filter = ComplementaryFilter::new(0.0);
        filter.update(&imu_at(0, accel, [0.0; 3]));
        let estimate = filter.update(&imu_at(1000, accel, [0.0; 3]));
        assert_approx(estimate.roll, angle);
        assert_approx(estimate.pitch, 0.0);
    }

    #[test]
    fn pure_accel_extracts_correct_pitch_angle() {
        // alpha=0.0: nose-down 20°: a_x = -G*sin(20°), remaining force in a_z.
        let angle = 20_f32.to_radians();
        let accel = [-G * angle.sin(), 0.0, G * angle.cos()];
        let mut filter = ComplementaryFilter::new(0.0);
        filter.update(&imu_at(0, accel, [0.0; 3]));
        let estimate = filter.update(&imu_at(1000, accel, [0.0; 3]));
        assert_approx(estimate.pitch, angle);
        assert_approx(estimate.roll, 0.0);
    }

    #[test]
    fn pure_gyro_integrates_roll_rate() {
        // alpha=1.0: output is entirely from gyro integration.
        // 1 rad/s roll rate for 1 ms → 0.001 rad.
        let rate = 1.0_f32;
        let dt_us = 1000_u64;
        let expected_roll = rate * (dt_us as f32 * 1e-6);
        let mut filter = ComplementaryFilter::new(1.0);
        filter.update(&imu_at(0, [0.0, 0.0, G], [0.0; 3]));
        let estimate = filter.update(&imu_at(dt_us, [0.0, 0.0, G], [rate, 0.0, 0.0]));
        assert_approx(estimate.roll, expected_roll);
    }

    #[test]
    fn gyro_rates_are_passed_through_directly() {
        let gyro = [0.1, -0.2, 0.3];
        let mut filter = ComplementaryFilter::new(0.98);
        filter.update(&imu_at(0, [0.0, 0.0, G], [0.0; 3]));
        let estimate = filter.update(&imu_at(1000, [0.0, 0.0, G], gyro));
        assert_approx(estimate.roll_rate,  gyro[0]);
        assert_approx(estimate.pitch_rate, gyro[1]);
        assert_approx(estimate.yaw_rate,   gyro[2]);
    }
}
