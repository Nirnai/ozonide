mod config;
mod registers;

pub use config::{AccelerometerPowerMode, Config};
pub use registers::*;

use ozonide_core::msgs::ImuData;
use ozonide_core::traits::ImuSource;
use embassy_time::{Instant, Timer};
use embedded_hal::spi::SpiDevice;
use embedded_hal_async::digital::Wait;

pub struct Icm42688p<SPI, DRDY> {
    spi: SPI,
    drdy: DRDY,
    config: Config,
    buf: [u8; 15],
}

impl<SPI: SpiDevice, DRDY: Wait> Icm42688p<SPI, DRDY> {
    pub fn new(spi: SPI, drdy: DRDY, config: Config) -> Self {
        Self { spi, drdy, config, buf: [0u8; 15] }
    }

    pub async fn init(&mut self) {
        let who_am_i = self.read_register(WHO_AM_I);
        if who_am_i != WHO_AM_I_VALUE {
            defmt::error!(
                "ICM42688p: WHO_AM_I mismatch: expected {}, got {}",
                WHO_AM_I_VALUE,
                who_am_i
            );
            panic!("ICM42688p initialization failed");
        }

        self.write_register(DEVICE_CONFIG, 0x01);
        Timer::after_micros(1_000).await;

        if self.config.sample_rate.requires_low_noise() {
            if let AccelerometerPowerMode::_LowPower = self.config.accelerometer_power_mode {
                defmt::error!("ODR > 500Hz requires Low Noise mode for accelerometer");
                panic!("Invalid config: LP mode not supported at this ODR");
            }
        }

        self.write_register(
            GYRO_CONFIG0,
            (self.config.gyroscope_range.fs_sel() << 5) | self.config.sample_rate.odr(),
        );
        self.write_register(
            ACCEL_CONFIG0,
            (self.config.accelerometer_range.fs_sel() << 5) | self.config.sample_rate.odr(),
        );

        Timer::after_micros(200).await;

        let pwr = (0b11 << 2) | self.config.accelerometer_power_mode.accel_mode();
        self.write_register(PWR_MGMT0, pwr);

        Timer::after_millis(30).await;

        self.write_register(INT_CONFIG1, 0b00000000);
        self.write_register(INT_CONFIG, 0b00000011);
        self.write_register(INT_SOURCE0, UI_DRDY_INT1_EN);

        defmt::info!("ICM42688P initialized");
    }

    pub async fn data_ready(&mut self) {
        self.drdy.wait_for_rising_edge().await.ok();
    }

    pub async fn read(&mut self) -> ImuData {
        let timestamp_us = Instant::now().as_micros();

        self.read_registers(IMU_DATA_1, 14);
        let data = &self.buf[1..15];

        let accel_sens = self.config.accelerometer_range.sensitivity();
        let gyro_sens = self.config.gyroscope_range.sensitivity();

        let temp_raw = i16::from_be_bytes([data[0], data[1]]);
        let temperature = (temp_raw as f32 / 132.48) + 25.0;

        let a_scale = 9.80665 / accel_sens;
        let g_scale = (core::f32::consts::PI / 180.0) / gyro_sens;

        ImuData {
            timestamp_us,
            linear_acceleration: [
                i16::from_be_bytes([data[2], data[3]]) as f32 * a_scale,
                i16::from_be_bytes([data[4], data[5]]) as f32 * a_scale,
                i16::from_be_bytes([data[6], data[7]]) as f32 * a_scale,
            ],
            angular_velocity: [
                i16::from_be_bytes([data[8], data[9]]) as f32 * g_scale,
                i16::from_be_bytes([data[10], data[11]]) as f32 * g_scale,
                i16::from_be_bytes([data[12], data[13]]) as f32 * g_scale,
            ],
            temperature,
        }
    }

    fn read_register(&mut self, reg: u8) -> u8 {
        self.buf[0] = reg | 0x80;
        self.buf[1] = 0x00;
        self.spi.transfer_in_place(&mut self.buf[..2]).ok();
        self.buf[1]
    }

    fn read_registers(&mut self, reg: u8, len: usize) {
        self.buf[0] = reg | 0x80;
        self.buf[1..=len].fill(0x00);
        self.spi.transfer_in_place(&mut self.buf[..=len]).ok();
    }

    fn write_register(&mut self, reg: u8, val: u8) {
        self.buf[0] = reg & 0x7F;
        self.buf[1] = val;
        self.spi.transfer_in_place(&mut self.buf[..2]).ok();
    }
}

impl<SPI: SpiDevice, DRDY: Wait> ImuSource for Icm42688p<SPI, DRDY> {
    async fn init(&mut self) {
        self.init().await;
    }

    async fn data_ready(&mut self) {
        self.data_ready().await;
    }

    async fn read(&mut self) -> ImuData {
        self.read().await
    }
}
