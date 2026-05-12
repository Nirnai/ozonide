//! Common data types used throughout the flight controller
//!
//! This module defines the core data structures for sensor readings,
//! vehicle state, and control outputs.

/// 3D vector for accelerometer, gyroscope, magnetometer data
#[derive(Debug, Clone, Copy)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Quaternion representation for attitude
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// IMU sensor readings
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub accel: Vec3,      // m/s²
    pub gyro: Vec3,       // rad/s
    pub timestamp_us: u64,
}

/// Complete vehicle state estimate
#[derive(Debug, Clone, Copy)]
pub struct VehicleState {
    pub position: Vec3,        // meters
    pub velocity: Vec3,        // m/s
    pub attitude: Quaternion,  // orientation
    pub angular_velocity: Vec3, // rad/s
    pub timestamp_us: u64,
}

/// Control outputs for motors/actuators
#[derive(Debug, Clone, Copy)]
pub struct ControlOutput {
    pub motors: [u16; 4],  // PWM values or DShot commands
}

/// Camera frame metadata
#[derive(Debug, Clone, Copy)]
pub struct FrameMetadata {
    pub width: u16,
    pub height: u16,
    pub format: ImageFormat,
    pub timestamp_us: u64,
}

#[derive(Debug, Clone, Copy)]
pub enum ImageFormat {
    Grayscale,
    Rgb565,
    Rgb888,
}
