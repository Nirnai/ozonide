pub mod icm42688p;
// pub use icm42688p::Icm42688p;

use crate::config::SensorConfig;
use crate::types::ImuData;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;


pub enum Imu<SPI, CS, INT> {
    Icm42688p(icm42688p::Icm42688p<SPI, CS, INT>),
}


impl<SPI, CS, INT, SpiE, PinE> Imu<SPI, CS, INT>
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = PinE>,
    INT: InputPin<Error = PinE>,
{
    pub fn read_int_status(&mut self) -> u8 {
        match self {
            Self::Icm42688p(driver) => driver.read_int_status(),
        }
    }

    pub fn read(&mut self) -> ImuData {
        match self {
            Self::Icm42688p(driver) => driver.read(),
        }
    }
}

pub fn create_imu<SPI, CS, INT, SpiE, PinE>(
    config: &SensorConfig,
    spi: SPI,
    cs: CS,
    int: INT,
) -> Result<Imu<SPI, CS, INT>, &'static str>
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    CS: OutputPin<Error = PinE>,
    INT: InputPin<Error = PinE>,
{
    match config.name {
        "icm42688p" => {
            let imu_config = icm42688p::Config {
                sample_rate: icm42688p::SampleRate::from_str(config.param("sample_rate")),
                gyroscope_range: icm42688p::GyroscopeRange::from_str(config.param("gyro_range")),
                accelerometer_range: icm42688p::AccelerometerRange::from_str(config.param("accel_range")),
                accelerometer_power_mode: icm42688p::AccelerometerPowerMode::from_str(
                    config.param("accel_power_mode"),
                ),
            };
            let mut imu = icm42688p::Icm42688p::new(spi, cs, int, imu_config);
            imu.init();
            Ok(Imu::Icm42688p(imu))
        }
        unknown => {
            defmt::error!("Unknown IMU driver: {}", unknown);
            Err("Unknown IMU driver")
        }
    }
}
