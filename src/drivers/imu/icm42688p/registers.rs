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

// ============================================================================
// Interrupt Configuration
// ============================================================================

// /// Interrupt status register (read to check which interrupt fired)
// pub const INT_STATUS: u8 = 0x2D;

/// Interrupt pin configuration (polarity, drive mode, latch/pulse)
pub const INT_CONFIG: u8 = 0x14;

// /// Interrupt timing configuration 0
// pub const INT_CONFIG0: u8 = 0x63;

// /// Interrupt timing configuration 1
// pub const INT_CONFIG1: u8 = 0x64;

/// Interrupt source routing to INT1 pin
pub const INT_SOURCE0: u8 = 0x65;

// /// Interrupt source routing to INT2 pin
// pub const INT_SOURCE1: u8 = 0x66;

// // INT_CONFIG bit definitions
// /// INT1 active high (1) or low (0)
// pub const INT1_MODE_SHIFT: u8 = 2;
// /// INT1 push-pull (0) or open-drain (1)
// pub const INT1_DRIVE_SHIFT: u8 = 1;
// /// INT1 pulsed (0) or latched (1)
// pub const INT1_POLARITY_SHIFT: u8 = 0;

// INT_SOURCE0 bit definitions (sources routed to INT1)
/// UI Data Ready interrupt enable on INT1
pub const UI_DRDY_INT1_EN: u8 = 1 << 3;
// /// FIFO threshold interrupt enable on INT1  
// pub const FIFO_THS_INT1_EN: u8 = 1 << 2;
// /// FIFO full interrupt enable on INT1
// pub const FIFO_FULL_INT1_EN: u8 = 1 << 1;