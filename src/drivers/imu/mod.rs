pub mod icm42688p;

use crate::config::SensorConfig;
use crate::types::ImuData;


pub trait Imu {
    fn read(&mut self) -> ImuData;
}


pub enum AnyImu<SPI, CS> {
    Icm42688p(icm42688p::Icm42688p<SPI, CS>),
}

impl<SPI, CS, SpiE, PinE> Imu for AnyImu<SPI, CS>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = SpiE>
        + embedded_hal::blocking::spi::Write<u8, Error = SpiE>,
    CS: embedded_hal::digital::v2::OutputPin<Error = PinE>,
{
    fn read(&mut self) -> ImuData {
        match self {
            Self::Icm42688p(d) => d.read(),
        }
    }
}

pub fn create_imu<SPI, CS, SpiE, PinE>(
    config: &SensorConfig,
    spi: SPI,
    cs: CS,
) -> Result<AnyImu<SPI, CS>, &'static str>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = SpiE>
        + embedded_hal::blocking::spi::Write<u8, Error = SpiE>,
    CS: embedded_hal::digital::v2::OutputPin<Error = PinE>,
{
    match config.name {
        "icm42688p" => {
            let imu_config = icm42688p::Config {
                gyro_range: icm42688p::GyroRange::from_str(config.param("gyro_range")),
                accel_range: icm42688p::AccelRange::from_str(config.param("accel_range")),
                sample_rate: icm42688p::SampleRate::from_str(config.param("sample_rate")),
                accel_power_mode: icm42688p::AccelPowerMode::from_str(
                    config.param("accel_power_mode"),
                ),
            };
            let mut imu = icm42688p::Icm42688p::new(spi, cs, imu_config);
            imu.init();
            Ok(AnyImu::Icm42688p(imu))
        }
        unknown => {
            defmt::error!("Unknown IMU driver: {}", unknown);
            Err("Unknown IMU driver")
        }
    }
}
