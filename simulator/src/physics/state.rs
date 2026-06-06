//! Vehicle state and state-derivative types for the RK4 integrator.

use std::ops::{Add, Mul};

use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};

/// Full rigid body state of the quadrotor.
///
/// All vectors are expressed in the **ENU world frame** (East-North-Up) unless
/// noted otherwise. The [`Default`] implementation places the vehicle at 1 m
/// altitude, level, and at rest with all motors stopped.
pub struct VehicleState {
    /// Position in the ENU world frame (m). `z` is altitude above ground.
    pub position: Vector3<f64>,
    /// Attitude as a unit quaternion representing the body → world rotation.
    /// Identity means the body frame axes align with ENU.
    pub attitude: UnitQuaternion<f64>,
    /// Linear velocity in the ENU world frame (m/s).
    pub linear_velocity: Vector3<f64>,
    /// Angular velocity in the **body frame** (rad/s).
    pub angular_velocity: Vector3<f64>,
    /// Rotor angular speeds \[FR, RL, FL, RR\] (rad/s).
    /// Integrated via the first-order motor lag model; not the same as commanded speed.
    pub motor_angular_velocity: Vector4<f64>,
}

/// Time derivative of [`VehicleState`], used as the intermediate quantity in the RK4 loop.
///
/// Each field is the instantaneous rate of change of the corresponding state
/// variable. `Add` and `Mul<f64>` are implemented so the weighted RK4 sum
/// `(k1 + 2·k2 + 2·k3 + k4)` can be expressed directly in code.
pub struct VehicleStateDot {
    /// d(position)/dt = linear velocity in ENU (m/s).
    pub linear_velocity: Vector3<f64>,
    /// d(attitude)/dt as a raw quaternion rate (quaternion/s).
    /// Computed as q̇ = ½ · q ⊗ \[0, ω_body\].
    pub attitude_rate: Quaternion<f64>,
    /// Linear acceleration in the ENU world frame (m/s²).
    pub linear_acceleration: Vector3<f64>,
    /// Angular acceleration in the body frame (rad/s²).
    pub angular_acceleration: Vector3<f64>,
    /// Rate of change of rotor angular speeds (rad/s²).
    pub motor_angular_acceleration: Vector4<f64>,
}


impl Default for VehicleState {
    fn default() -> Self {
        Self {
            position: Vector3::new(0.0, 0.0, 1.0),
            attitude: UnitQuaternion::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            motor_angular_velocity: Vector4::zeros(),
        }
    }
}


impl Add for VehicleStateDot {
    type Output = Self;
    fn add(self, other: Self) -> Self::Output {
        Self {
            linear_velocity: self.linear_velocity + other.linear_velocity,
            attitude_rate: self.attitude_rate + other.attitude_rate,
            linear_acceleration: self.linear_acceleration + other.linear_acceleration,
            angular_acceleration: self.angular_acceleration + other.angular_acceleration,
            motor_angular_acceleration: self.motor_angular_acceleration
                + other.motor_angular_acceleration,
        }
    }
}

impl Mul<f64> for VehicleStateDot {
    type Output = Self;
    fn mul(self, factor: f64) -> Self::Output {
        Self {
            linear_velocity: factor * self.linear_velocity,
            attitude_rate: factor * self.attitude_rate,
            linear_acceleration: factor * self.linear_acceleration,
            angular_acceleration: factor * self.angular_acceleration,
            motor_angular_acceleration: factor * self.motor_angular_acceleration,
        }
    }
}
