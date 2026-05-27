use embedded_hal_bus::spi::ExclusiveDevice;
use embassy_time::Delay;
use embassy_stm32::spi::Spi;
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_time::Timer;
use core::sync::atomic::Ordering;

use crate::drivers::imu::icm42688p::Icm42688p;
use crate::topics::{IMU_TOPIC, IMU_PUB_COUNT};

type ImuSpi = ExclusiveDevice<Spi<'static, Blocking, embassy_stm32::spi::mode::Master>, Output<'static>, Delay>;
type ImuDrdy = ExtiInput<'static, Async>;

#[embassy_executor::task]
pub async fn imu_task(imu_driver: &'static mut Icm42688p<ImuSpi, ImuDrdy>) {
    imu_driver.init().await;
    defmt::info!("IMU driver ready, entering sample loop");
    let tx = IMU_TOPIC.sender();
    loop {
        imu_driver.data_ready().await;
        let sample = imu_driver.read().await;
        tx.send(sample);
        IMU_PUB_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
pub async fn imu_rate_monitor() {
    let mut prev_imu = 0u32;
    loop {
        Timer::after_secs(1).await;
        let imu_count = IMU_PUB_COUNT.load(Ordering::Relaxed);
        defmt::info!("IMU Topic is running at : {} Hz", imu_count - prev_imu);
        prev_imu = imu_count;
    }
}
