use std::ops::{Add, Mul};

use nalgebra::{Quaternion, UnitQuaternion, Vector3, Vector4};
pub struct VehicleState {
    pub position: Vector3<f64>,
    pub attitude: UnitQuaternion<f64>,
    pub linear_velocity: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub motor_angular_velocity: Vector4<f64>,
}

pub struct VehicleStateDot {
    pub linear_velocity: Vector3<f64>,
    pub attitude_rate: Quaternion<f64>,
    pub linear_acceleration: Vector3<f64>,
    pub angular_acceleration: Vector3<f64>,
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


