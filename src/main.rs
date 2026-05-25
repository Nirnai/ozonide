#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::spi::Spi;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use static_cell::StaticCell;

use drivers::imu::icm42688p::Icm42688p;

mod board;
mod drivers;
mod executors;
mod msgs;

type ImuSpi = ExclusiveDevice<Spi<'static, Blocking, embassy_stm32::spi::mode::Master>, Output<'static>, Delay>;
type ImuDrdy = ExtiInput<'static, Async>;

static IMU: StaticCell<Icm42688p<ImuSpi, ImuDrdy>> = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Ozonide flight controller starting...");

    let board = board::Board::init();

    let imu_config = drivers::imu::icm42688p::Config {
        sample_rate: drivers::imu::icm42688p::SampleRate::_1000_hz,
        gyroscope_range: drivers::imu::icm42688p::GyroscopeRange::_2000_dps,
        accelerometer_range: drivers::imu::icm42688p::AccelerometerRange::_16_g,
        accelerometer_power_mode: drivers::imu::icm42688p::AccelerometerPowerMode::_LowNoise,
    };

    let imu = IMU.init(Icm42688p::new(
        ExclusiveDevice::new(board.imu_spi, board.imu_chip_select, Delay).unwrap(),
        board.imu_data_ready,
        imu_config,
    ));

    let urgent_tasks = executors::start_urgent_executor();
    urgent_tasks.spawn(imu_measurement(imu).unwrap());

    executors::run_remaining_tasks(|_spawner| {
        // future tasks (control loop, comms) spawned here
    });
}

#[embassy_executor::task]
async fn imu_measurement(imu: &'static mut Icm42688p<ImuSpi, ImuDrdy>) {
    imu.init().await;
    defmt::info!("IMU ready, entering sample loop");

    loop {
        imu.data_ready().await;
        let sample = imu.read().await;
        defmt::info!(
            "IMU [{}us]: ax={} ay={} az={} gx={} gy={} gz={} t={}",
            sample.timestamp_us,
            sample.linear_acceleration[0],
            sample.linear_acceleration[1],
            sample.linear_acceleration[2],
            sample.angular_velocity[0],
            sample.angular_velocity[1],
            sample.angular_velocity[2],
            sample.temperature,
        );
    }
}
