//! Placeholder state estimator that forwards ground-truth pose and velocity.
//!
//! This is a **temporary stand-in for the UKF**. It does no fusion: it simply
//! assembles a [`VehicleState`] from whatever measurements are available this
//! cycle, taking pose from the (SITL) ground-truth reference and the inertial
//! rate signals straight off the IMU.
//!
//! ## Field provenance
//!
//! | `VehicleState` field         | Source                          | Noisy? |
//! |------------------------------|---------------------------------|--------|
//! | `attitude`                   | ground truth                    | clean  |
//! | `position`, `linear_velocity`| ground truth                    | clean  |
//! | `angular_velocity`           | raw IMU gyro passthrough        | **yes**|
//! | `specific_force`             | raw IMU accel passthrough       | **yes**|
//! | `motor_speed`                | actuator telemetry              | (sim)  |
//!
//! The inertial rates are deliberately left raw: the INDI rate loop consumes
//! them directly and must keep seeing real sensor noise/bias. Only the *pose*
//! is idealised — exactly the quantities a navigation filter would estimate.
//!
//! When the UKF lands it replaces this type wholesale: it will fuse IMU, camera,
//! and actuator feedback to produce all of these fields, and the ground-truth
//! input disappears.

use crate::{
    msgs::{StateValidity, VehicleState},
    traits::{SensorData, StateEstimator},
};

/// Stateless passthrough estimator. See module docs.
pub struct PassthroughStateEstimator;

impl PassthroughStateEstimator {
    pub fn new() -> Self {
        Self
    }
}

impl Default for PassthroughStateEstimator {
    fn default() -> Self {
        Self::new()
    }
}

impl StateEstimator for PassthroughStateEstimator {
    fn update(&mut self, sensor_data: &SensorData<'_>) -> VehicleState {
        let Some(imu) = sensor_data.imu else {
            return VehicleState::default();
        };

        let mut state = VehicleState {
            timestamp_us: imu.timestamp_us,
            valid: StateValidity::NONE,
            angular_velocity: imu.angular_velocity,
            specific_force: imu.specific_force,
            ..VehicleState::default()
        };

        // Pose from the ground-truth reference (UKF output, eventually).
        if let Some(gt) = sensor_data.ground_truth {
            state.attitude = gt.attitude;
            state.position = gt.position;
            state.linear_velocity = gt.linear_velocity;
            state.valid = state
                .valid
                .with(StateValidity::ATTITUDE)
                .with(StateValidity::POSITION)
                .with(StateValidity::VELOCITY);
        }

        // Motor speed from ESC telemetry (simulator motor model in SITL).
        if let Some(actuator) = sensor_data.actuator {
            state.motor_speed = actuator.motor_speed;
            state.valid = state.valid.with(StateValidity::MOTOR_SPEED);
        }

        state
    }

    fn reset(&mut self) {
        // Stateless — nothing to reset.
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msgs::{ActuatorTelemetry, GroundTruthState, ImuData};

    fn imu() -> ImuData {
        ImuData {
            timestamp_us: 1000,
            specific_force: [0.1, -0.2, 9.7],
            angular_velocity: [0.3, 0.4, 0.5],
            temperature: 25.0,
        }
    }

    fn ground_truth() -> GroundTruthState {
        GroundTruthState {
            timestamp_us: 1000,
            position: [1.0, 2.0, 3.0],
            linear_velocity: [0.5, -0.5, 0.1],
            attitude: [0.0, 0.0, 0.0, 1.0],
        }
    }

    #[test]
    fn forwards_ground_truth_pose_and_raw_imu_rates() {
        let imu = imu();
        let gt = ground_truth();
        let data = SensorData {
            imu: Some(&imu),
            actuator: None,
            battery: None,
            ground_truth: Some(&gt),
        };
        let state = PassthroughStateEstimator::new().update(&data);

        assert_eq!(state.position, gt.position);
        assert_eq!(state.linear_velocity, gt.linear_velocity);
        assert_eq!(state.attitude, gt.attitude);
        // Inertial rates remain the raw IMU values.
        assert_eq!(state.angular_velocity, imu.angular_velocity);
        assert_eq!(state.specific_force, imu.specific_force);
        assert_eq!(state.timestamp_us, imu.timestamp_us);

        assert!(state.valid.contains(StateValidity::ATTITUDE));
        assert!(state.valid.contains(StateValidity::POSITION));
        assert!(state.valid.contains(StateValidity::VELOCITY));
        assert!(!state.valid.contains(StateValidity::MOTOR_SPEED));
    }

    #[test]
    fn pose_invalid_until_ground_truth_arrives() {
        let imu = imu();
        let data = SensorData {
            imu: Some(&imu),
            actuator: None,
            battery: None,
            ground_truth: None,
        };
        let state = PassthroughStateEstimator::new().update(&data);

        assert!(!state.valid.contains(StateValidity::ATTITUDE));
        assert!(!state.valid.contains(StateValidity::POSITION));
        assert!(!state.valid.contains(StateValidity::VELOCITY));
        // Rates still pass through.
        assert_eq!(state.angular_velocity, imu.angular_velocity);
    }

    #[test]
    fn motor_speed_flagged_when_telemetry_present() {
        let imu = imu();
        let gt = ground_truth();
        let telem = ActuatorTelemetry { timestamp_us: 1000, motor_speed: [100.0; 4] };
        let data = SensorData {
            imu: Some(&imu),
            actuator: Some(&telem),
            battery: None,
            ground_truth: Some(&gt),
        };
        let state = PassthroughStateEstimator::new().update(&data);
        assert!(state.valid.contains(StateValidity::MOTOR_SPEED));
        assert_eq!(state.motor_speed, telem.motor_speed);
    }

    #[test]
    fn no_imu_yields_default_state() {
        let data = SensorData { imu: None, actuator: None, battery: None, ground_truth: None };
        let state = PassthroughStateEstimator::new().update(&data);
        assert!(!state.valid.contains(StateValidity::ATTITUDE));
        assert_eq!(state.timestamp_us, 0);
    }
}
