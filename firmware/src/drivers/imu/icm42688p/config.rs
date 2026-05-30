#[derive(Clone, Copy)]
#[allow(non_camel_case_types)]
pub enum SampleRate {
    _12_5_hz,
    _25_hz,
    _50_hz,
    _100_hz,
    _200_hz,
    _500_hz,
    _1000_hz,
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
    _LowNoise,
    _LowPower,
}

pub struct Config {
    pub sample_rate: SampleRate,
    pub gyroscope_range: GyroscopeRange,
    pub accelerometer_range: AccelerometerRange,
    pub accelerometer_power_mode: AccelerometerPowerMode,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sample_rate: SampleRate::_1000_hz,
            gyroscope_range: GyroscopeRange::_2000_dps,
            accelerometer_range: AccelerometerRange::_16_g,
            accelerometer_power_mode: AccelerometerPowerMode::_LowNoise,
        }
    }
}

impl GyroscopeRange {
    pub(super) fn sensitivity(&self) -> f32 {
        match self {
            Self::_250_dps => 131.0,
            Self::_500_dps => 65.5,
            Self::_1000_dps => 32.8,
            Self::_2000_dps => 16.4,
        }
    }

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
    pub(super) fn sensitivity(&self) -> f32 {
        match self {
            Self::_2_g => 16384.0,
            Self::_4_g => 8192.0,
            Self::_8_g => 4096.0,
            Self::_16_g => 2048.0,
        }
    }

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

    pub(super) fn requires_low_noise(&self) -> bool {
        matches!(
            self,
            Self::_1000_hz
                | Self::_2000_hz
                | Self::_4000_hz
                | Self::_8000_hz
                | Self::_16000_hz
                | Self::_32000_hz
        )
    }
}

impl AccelerometerPowerMode {
    pub(super) fn accel_mode(&self) -> u8 {
        match self {
            Self::_LowNoise => 0b11,
            Self::_LowPower => 0b10,
        }
    }
}
