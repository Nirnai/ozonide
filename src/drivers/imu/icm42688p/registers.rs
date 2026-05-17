// ============================================================================
// Device Identification
// ============================================================================

/// Device identification register (should read 0x47 for ICM-42688-P)
pub const WHO_AM_I: u8 = 0x75;
pub const WHO_AM_I_VALUE: u8 = 0x47;

// ============================================================================
// Configuration Registers
// ============================================================================

/// Device configuration register
pub const DEVICE_CONFIG: u8 = 0x11;

/// Gyroscope configuration (full-scale range and ODR)
pub const GYRO_CONFIG0: u8 = 0x4F;

/// Accelerometer configuration (full-scale range and ODR)
pub const ACCEL_CONFIG0: u8 = 0x50;

// ============================================================================
// Data Registers
// ============================================================================

/// Accelerometer X-axis high byte (start of 6-byte accel data)
pub const ACCEL_DATA_X1: u8 = 0x1F;

/// Gyroscope X-axis high byte (start of 6-byte gyro data)
pub const GYRO_DATA_X1: u8 = 0x25;

// ============================================================================
// Power Management
// ============================================================================

/// Power management register (gyro and accel modes)
pub const PWR_MGMT0: u8 = 0x4E;