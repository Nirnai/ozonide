//! Inertial Measurement Unit (IMU) drivers
//!
//! Provides a common interface for 6-DOF IMU sensors.
//! Implementations should support at least 100Hz sampling rate.

use crate::types::{ImuData, Vec3};

/// Common interface for IMU sensors
pub trait Imu {
    type Error;

    /// Read accelerometer data in m/s²
    fn read_accel(&mut self) -> Result<Vec3, Self::Error>;

    /// Read gyroscope data in rad/s
    fn read_gyro(&mut self) -> Result<Vec3, Self::Error>;

    /// Read both accel and gyro in one operation (more efficient)
    fn read(&mut self) -> Result<ImuData, Self::Error>;

    /// Configure sensor ranges and sample rate
    fn configure(&mut self, config: &ImuConfig) -> Result<(), Self::Error>;

    /// Check if new data is available
    fn data_ready(&mut self) -> Result<bool, Self::Error>;
}

/// IMU configuration parameters
#[derive(Debug, Clone, Copy)]
pub struct ImuConfig {
    pub accel_range: AccelRange,
    pub gyro_range: GyroRange,
    pub sample_rate_hz: u16,
    pub low_pass_filter: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum AccelRange {
    G2,   // ±2g
    G4,   // ±4g
    G8,   // ±8g
    G16,  // ±16g
}

#[derive(Debug, Clone, Copy)]
pub enum GyroRange {
    Dps250,   // ±250 degrees/sec
    Dps500,   // ±500 degrees/sec
    Dps1000,  // ±1000 degrees/sec
    Dps2000,  // ±2000 degrees/sec
}

// TODO: Implement for specific IMU chips
// mod mpu6050;
// mod icm20689;
// mod bmi088;
