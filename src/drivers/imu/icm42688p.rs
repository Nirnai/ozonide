use crate::types::ImuData;
use crate::types::Vector3;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

// Register addresses
const REG_DEVICE_CONFIG: u8 = 0x11;
const REG_PWR_MGMT0: u8 = 0x4E;
const REG_GYRO_CONFIG0: u8 = 0x4F;
const REG_ACCEL_CONFIG0: u8 = 0x50;
const REG_ACCEL_DATA_X1: u8 = 0x1F;
const REG_GYRO_DATA_X1: u8 = 0x25;
const REG_WHO_AM_I: u8 = 0x75;

const WHO_AM_I_VALUE: u8 = 0x47;

#[derive(Clone, Copy)]
pub enum GyroRange {
    Dps250,
    Dps500,
    Dps1000,
    Dps2000,
}

impl GyroRange {
    pub fn from_str(s: &str) -> Self {
        match s {
            "250_dps" => Self::Dps250,
            "500_dps" => Self::Dps500,
            "1000_dps" => Self::Dps1000,
            "2000_dps" => Self::Dps2000,
            _ => panic!("Invalid gyro range: {}", s),
        }
    }

    /// LSB/°/s from datasheet Table 1
    fn sensitivity(&self) -> f32 {
        match self {
            Self::Dps250 => 131.0,
            Self::Dps500 => 65.5,
            Self::Dps1000 => 32.8,
            Self::Dps2000 => 16.4,
        }
    }

    /// Bits [7:5] of GYRO_CONFIG0
    fn fs_sel(&self) -> u8 {
        match self {
            Self::Dps2000 => 0b000,
            Self::Dps1000 => 0b001,
            Self::Dps500 => 0b010,
            Self::Dps250 => 0b011,
        }
    }
}

#[derive(Clone, Copy)]
pub enum AccelRange {
    G2,
    G4,
    G8,
    G16,
}

impl AccelRange {
    pub fn from_str(s: &str) -> Self {
        match s {
            "2_g" => Self::G2,
            "4_g" => Self::G4,
            "8_g" => Self::G8,
            "16_g" => Self::G16,
            _ => panic!("Invalid accel range: {}", s),
        }
    }

    /// LSB/g from datasheet Table 2
    fn sensitivity(&self) -> f32 {
        match self {
            Self::G2 => 16384.0,
            Self::G4 => 8192.0,
            Self::G8 => 4096.0,
            Self::G16 => 2048.0,
        }
    }

    /// Bits [7:5] of ACCEL_CONFIG0
    fn fs_sel(&self) -> u8 {
        match self {
            Self::G16 => 0b000,
            Self::G8 => 0b001,
            Self::G4 => 0b010,
            Self::G2 => 0b011,
        }
    }
}

#[derive(Clone, Copy)]
pub enum SampleRate {
    // Low Power or Low Noise mode
    Hz12_5,
    Hz25,
    Hz50,
    Hz100,
    Hz200,
    Hz500,
    Hz1000,
    // Low Noise mode only
    Hz2000,
    Hz4000,
    Hz8000,
    Hz16000,
    Hz32000,
}

impl SampleRate {
    pub fn from_str(s: &str) -> Self {
        match s {
            "12.5_hz" => Self::Hz12_5,
            "25_hz" => Self::Hz25,
            "50_hz" => Self::Hz50,
            "100_hz" => Self::Hz100,
            "200_hz" => Self::Hz200,
            "500_hz" => Self::Hz500,
            "1000_hz" => Self::Hz1000,
            "2000_hz" => Self::Hz2000,
            "4000_hz" => Self::Hz4000,
            "8000_hz" => Self::Hz8000,
            "16000_hz" => Self::Hz16000,
            "32000_hz" => Self::Hz32000,
            _ => panic!("Invalid sample rate: {}", s),
        }
    }

    /// Bits [3:0] of GYRO_CONFIG0 / ACCEL_CONFIG0
    fn odr(&self) -> u8 {
        match self {
            Self::Hz32000 => 0x01,
            Self::Hz16000 => 0x02,
            Self::Hz8000 => 0x03,
            Self::Hz4000 => 0x04,
            Self::Hz2000 => 0x05,
            Self::Hz1000 => 0x06,
            Self::Hz200 => 0x07,
            Self::Hz100 => 0x08,
            Self::Hz50 => 0x09,
            Self::Hz25 => 0x0A,
            Self::Hz12_5 => 0x0B,
            Self::Hz500 => 0x0F,
        }
    }

    /// Returns true if this ODR requires Low Noise mode
    fn requires_low_noise(&self) -> bool {
        match self {
            Self::Hz1000
            | Self::Hz2000
            | Self::Hz4000
            | Self::Hz8000
            | Self::Hz16000
            | Self::Hz32000 => true,
            _ => false,
        }
    }
}

#[derive(Clone, Copy)]
pub enum AccelPowerMode {
    LowNoise,
    LowPower,
}

impl AccelPowerMode {
    pub fn from_str(s: &str) -> Self {
        match s {
            "low_noise" => Self::LowNoise,
            "low_power" => Self::LowPower,
            _ => panic!("Invalid accel power mode: {}", s),
        }
    }

    /// Bits [1:0] of PWR_MGMT0
    fn accel_mode(&self) -> u8 {
        match self {
            Self::LowNoise => 0b11,
            Self::LowPower => 0b10,
        }
    }
}

pub struct Config {
    pub gyro_range: GyroRange,
    pub accel_range: AccelRange,
    pub sample_rate: SampleRate,
    pub accel_power_mode: AccelPowerMode,
}

pub struct Icm42688p<SPI, CS> {
    spi: SPI,
    cs: CS,
    config: Config,
}

impl<SPI, CS, SpiE, PinE> Icm42688p<SPI, CS>
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = PinE>,
{
    pub fn new(spi: SPI, cs: CS, config: Config) -> Self {
        Self { spi, cs, config }
    }

    pub fn init(&mut self) {
        // Verify WHO_AM_I
        let who_am_i = self.read_register(REG_WHO_AM_I);
        defmt::info!("ICM42688P WHO_AM_I: {:#04x}", who_am_i);
        if who_am_i != WHO_AM_I_VALUE {
            defmt::error!("ICM42688P WHO_AM_I mismatch: expected {:#04x}, got {:#04x}",
                WHO_AM_I_VALUE, who_am_i);
        }

        // Soft reset
        self.write_register(REG_DEVICE_CONFIG, 0x01);
        cortex_m::asm::delay(480_000); // ~1ms at 480MHz

        // Validate: if ODR > 500Hz, accel must be in Low Noise mode
        if self.config.sample_rate.requires_low_noise() {
            match self.config.accel_power_mode {
                AccelPowerMode::LowPower => {
                    defmt::error!("ODR > 500Hz requires Low Noise mode for accelerometer");
                    panic!("Invalid config: LP mode not supported at this ODR");
                }
                _ => {}
            }
        }

        // Configure GYRO_CONFIG0: FS_SEL [7:5] | ODR [3:0]
        self.write_register(
            REG_GYRO_CONFIG0,
            (self.config.gyro_range.fs_sel() << 5) | self.config.sample_rate.odr(),
        );

        // Configure ACCEL_CONFIG0: FS_SEL [7:5] | ODR [3:0]
        self.write_register(
            REG_ACCEL_CONFIG0,
            (self.config.accel_range.fs_sel() << 5) | self.config.sample_rate.odr(),
        );

        // Wait 200µs before writing PWR_MGMT0
        cortex_m::asm::delay(96_000);

        // PWR_MGMT0: GYRO_MODE [3:2] = LN (0b11), ACCEL_MODE [1:0] from config
        let pwr = (0b11 << 2) | self.config.accel_power_mode.accel_mode();
        self.write_register(REG_PWR_MGMT0, pwr);

        // Wait for gyro startup (30ms typical per datasheet)
        cortex_m::asm::delay(480_000 * 30);

        defmt::info!("ICM42688P initialized");
    }

    pub fn read(&mut self) -> ImuData {
        let mut accel_buf = [0u8; 6];
        let mut gyro_buf = [0u8; 6];
        self.read_registers(REG_ACCEL_DATA_X1, &mut accel_buf);
        self.read_registers(REG_GYRO_DATA_X1, &mut gyro_buf);

        let accel_scale = 1.0 / self.config.accel_range.sensitivity();
        let gyro_scale = 1.0 / self.config.gyro_range.sensitivity();

        ImuData {
            timestamp_us: 0, // TODO: wire up a timer
            linear_acceleration: Vector3 {
                x: (i16::from_be_bytes([accel_buf[0], accel_buf[1]]) as f32) * accel_scale,
                y: (i16::from_be_bytes([accel_buf[2], accel_buf[3]]) as f32) * accel_scale,
                z: (i16::from_be_bytes([accel_buf[4], accel_buf[5]]) as f32) * accel_scale,
            },
            angular_velocity: Vector3 {
                x: (i16::from_be_bytes([gyro_buf[0], gyro_buf[1]]) as f32) * gyro_scale,
                y: (i16::from_be_bytes([gyro_buf[2], gyro_buf[3]]) as f32) * gyro_scale,
                z: (i16::from_be_bytes([gyro_buf[4], gyro_buf[5]]) as f32) * gyro_scale,
            },
        }
    }

    fn read_register(&mut self, reg: u8) -> u8 {
        let mut buf = [reg | 0x80, 0x00];
        let _ = self.cs.set_low();
        let _ = self.spi.transfer(&mut buf);
        let _ = self.cs.set_high();
        buf[1]
    }

    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) {
        let _ = self.cs.set_low();
        let _ = self.spi.write(&[reg | 0x80]);
        let _ = self.spi.transfer(buf);
        let _ = self.cs.set_high();
    }

    fn write_register(&mut self, reg: u8, val: u8) {
        let _ = self.cs.set_low();
        let _ = self.spi.write(&[reg & 0x7F, val]);
        let _ = self.cs.set_high();
    }
}
