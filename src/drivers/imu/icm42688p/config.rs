// ============================================================================
// Type Definitions
// ============================================================================

#[derive(Clone, Copy)]
#[allow(non_camel_case_types)]
pub enum SampleRate {
    // Low Power or Low Noise mode
    _12_5_hz,
    _25_hz,
    _50_hz,
    _100_hz,
    _200_hz,
    _500_hz,
    _1000_hz,
    // Low Noise mode only
    _2000_hz,
    _4000_hz,
    _8000_hz,
    _16000_hz,
    _32000_hz,
}

#[derive(Clone, Copy)]
#[allow(non_camel_case_types)]
pub enum GyroscopeRange {
    _250_dps,
    _500_dps,
    _1000_dps,
    _2000_dps,
}

#[derive(Clone, Copy)]
#[allow(non_camel_case_types)]
pub enum AccelerometerRange {
    _2_g,
    _4_g,
    _8_g,
    _16_g,
}

#[derive(Clone, Copy)]
pub enum AccelerometerPowerMode {
    LowNoise,
    LowPower,
}

pub struct Config {
    pub sample_rate: SampleRate,
    pub gyroscope_range: GyroscopeRange,
    pub accelerometer_range: AccelerometerRange,
    pub accelerometer_power_mode: AccelerometerPowerMode,   
}


// ============================================================================
// Implementations
// ============================================================================

impl GyroscopeRange {
    pub fn from_str(s: &str) -> Self {
        match s {
            "250_dps" => Self::_250_dps,
            "500_dps" => Self::_500_dps,
            "1000_dps" => Self::_1000_dps,
            "2000_dps" => Self::_2000_dps,
            _ => panic!("Invalid gyro range: {}", s),
        }
    }

    /// LSB/°/s from datasheet Table 1
    pub(super) fn sensitivity(&self) -> f32 {
        match self {
            Self::_250_dps => 131.0,
            Self::_500_dps => 65.5,
            Self::_1000_dps => 32.8,
            Self::_2000_dps => 16.4,
        }
    }

    /// Bits [7:5] of GYRO_CONFIG0
    pub(super) fn fs_sel(&self) -> u8 {
        match self {
            Self::_2000_dps => 0b000,
            Self::_1000_dps => 0b001,
            Self::_500_dps => 0b010,
            Self::_250_dps => 0b011,
        }
    }
}

impl AccelerometerRange {
    pub fn from_str(s: &str) -> Self {
        match s {
            "2_g" => Self::_2_g,
            "4_g" => Self::_4_g,
            "8_g" => Self::_8_g,
            "16_g" => Self::_16_g,
            _ => panic!("Invalid accel range: {}", s),
        }
    }

    /// LSB/g from datasheet Table 2
    pub(super) fn sensitivity(&self) -> f32 {
        match self {
            Self::_2_g => 16384.0,
            Self::_4_g => 8192.0,
            Self::_8_g => 4096.0,
            Self::_16_g => 2048.0,
        }
    }

    /// Bits [7:5] of ACCEL_CONFIG0
    pub(super) fn fs_sel(&self) -> u8 {
        match self {
            Self::_16_g => 0b000,
            Self::_8_g => 0b001,
            Self::_4_g => 0b010,
            Self::_2_g => 0b011,
        }
    }
}

impl SampleRate {
    pub fn from_str(s: &str) -> Self {
        match s {
            "12.5_hz" => Self::_12_5_hz,
            "25_hz" => Self::_25_hz,
            "50_hz" => Self::_50_hz,
            "100_hz" => Self::_100_hz,
            "200_hz" => Self::_200_hz,
            "500_hz" => Self::_500_hz,
            "1000_hz" => Self::_1000_hz,
            "2000_hz" => Self::_2000_hz,
            "4000_hz" => Self::_4000_hz,
            "8000_hz" => Self::_8000_hz,
            "16000_hz" => Self::_16000_hz,
            "32000_hz" => Self::_32000_hz,
            _ => panic!("Invalid sample rate: {}", s),
        }
    }

    /// Bits [3:0] of GYRO_CONFIG0 / ACCEL_CONFIG0
    pub(super) fn odr(&self) -> u8 {
        match self {
            Self::_32000_hz => 0x01,
            Self::_16000_hz => 0x02,
            Self::_8000_hz => 0x03,
            Self::_4000_hz => 0x04,
            Self::_2000_hz => 0x05,
            Self::_1000_hz => 0x06,
            Self::_500_hz => 0x07,
            Self::_200_hz => 0x08,
            Self::_100_hz => 0x09,
            Self::_50_hz => 0x0A,
            Self::_25_hz => 0x0B,
            Self::_12_5_hz => 0x0C,
        }
    }

    /// Returns true if this ODR requires Low Noise mode
    pub(super) fn requires_low_noise(&self) -> bool {
        matches!(self, 
            Self::_1000_hz | Self::_2000_hz | Self::_4000_hz | 
            Self::_8000_hz | Self::_16000_hz | Self::_32000_hz
        )
    }
}

impl AccelerometerPowerMode {
    pub fn from_str(s: &str) -> Self {
        match s {
            "low_noise" => Self::LowNoise,
            "low_power" => Self::LowPower,
            _ => panic!("Invalid accel power mode: {}", s),
        }
    }

    /// Bits [1:0] of PWR_MGMT0
    pub(super) fn accel_mode(&self) -> u8 {
        match self {
            Self::LowNoise => 0b11,
            Self::LowPower => 0b10,
        }
    }
}