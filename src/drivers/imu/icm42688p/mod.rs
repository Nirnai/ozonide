// Declare submodules
mod config;
mod registers;

// Re-export public types
pub use config::{
    Config,
    GyroscopeRange,
    AccelerometerRange,
    AccelerometerPowerMode,
    SampleRate,
};

pub use registers::*;


// Imports for internal use
use crate::types::{ImuData, Vector3};
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

pub struct Icm42688p<SPI, CS, INT> {
    spi: SPI,
    cs: CS,
    _int: INT,
    config: Config,
}

impl<SPI, CS, INT, SpiError, PinError> Icm42688p<SPI, CS, INT>
where
    SPI: Transfer<u8, Error = SpiError> + Write<u8, Error = SpiError>,
    CS: OutputPin<Error = PinError>,
    INT: InputPin<Error = PinError>,
{
    pub fn new(spi: SPI, cs: CS, int: INT, config: Config) -> Self {
        Self { spi, cs, _int: int, config }
    }

    pub fn init(&mut self) {
        let who_am_i = self.read_register(WHO_AM_I);
        if who_am_i != WHO_AM_I_VALUE {
            defmt::error!("ICM42688p: WHO_AM_I mismatch: expected {}, got {}", WHO_AM_I_VALUE, who_am_i);
            panic!("ICM42688p initialization failed");
        }

        // Soft reset — 1ms minimum before writing registers (datasheet §9.17)
        self.write_register(DEVICE_CONFIG, 0x01);
        crate::utils::SystemTime::delay(1_000);

        // Validate: if ODR > 500Hz, accel must be in Low Noise mode
        if self.config.sample_rate.requires_low_noise() {
            match self.config.accelerometer_power_mode {
                AccelerometerPowerMode::LowPower => {
                    defmt::error!("ODR > 500Hz requires Low Noise mode for accelerometer");
                    panic!("Invalid config: LP mode not supported at this ODR");
                }
                _ => {}
            }
        }
        // Configure GYRO_CONFIG0: FS_SEL [7:5] | ODR [3:0]
        self.write_register(
            GYRO_CONFIG0,
            (self.config.gyroscope_range.fs_sel() << 5) | self.config.sample_rate.odr(),
        );
        // Configure ACCEL_CONFIG0: FS_SEL [7:5] | ODR [3:0]
        self.write_register(
            ACCEL_CONFIG0,
            (self.config.accelerometer_range.fs_sel() << 5) | self.config.sample_rate.odr(),
        );
        // Wait 200µs before writing PWR_MGMT0
        crate::utils::SystemTime::delay(200);
        // PWR_MGMT0: GYRO_MODE [3:2] = LN (0b11), ACCEL_MODE [1:0] from config
        let pwr = (0b11 << 2) | self.config.accelerometer_power_mode.accel_mode();
        self.write_register(PWR_MGMT0, pwr);

        // Wait for gyro startup (30ms typical per datasheet)
        crate::utils::SystemTime::delay(30_000);

        // Configure interrupts
        // INT_CONFIG1: Clear INT_ASYNC_RESET (bit 4). Datasheet requires this after every
        // power-on or soft reset — leaving it at the default of 1 causes improper INT pin behavior.
        self.write_register(INT_CONFIG1, 0b00000000);
        // INT_CONFIG: INT1 active high, push-pull, pulsed
        // bit2=0 → pulsed mode: INT1 pulses HIGH briefly (~15 µs at 1 kHz, scope-verified).
        // bit1=1 → push-pull drive. bit0=1 → active high.
        self.write_register(INT_CONFIG, 0b00000011); // bit2=pulsed, bit1=push-pull, bit0=active-high

        // INT_SOURCE0: Route UI Data Ready to INT1
        self.write_register(INT_SOURCE0, UI_DRDY_INT1_EN);
        defmt::info!("ICM42688P initialized with INT1 data ready");
    }

    pub fn read_int_status(&mut self) -> u8 {
        self.read_register(INT_STATUS)
    }

    pub fn read(&mut self) -> ImuData {
        let mut accel_buf = [0u8; 6];
        let mut gyro_buf = [0u8; 6];

        self.read_registers(ACCEL_DATA_X1, &mut accel_buf);
        self.read_registers(GYRO_DATA_X1, &mut gyro_buf);

        let accel_scale = 1.0 / self.config.accelerometer_range.sensitivity();
        let gyro_scale = 1.0 / self.config.gyroscope_range.sensitivity();

        return ImuData {
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
        };
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