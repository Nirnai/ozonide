use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::spi::Spi;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;

use crate::drivers::imu::icm42688p::Icm42688p;
use ozonide_core::topics::IMU_TOPIC;

type ImuSpi =
    ExclusiveDevice<Spi<'static, Blocking, embassy_stm32::spi::mode::Master>, Output<'static>, Delay>;
type ImuDrdy = ExtiInput<'static, Async>;

#[embassy_executor::task]
pub async fn imu_task(imu: &'static mut Icm42688p<ImuSpi, ImuDrdy>) {
    ozonide_core::tasks::imu_task(imu).await;
}

#[embassy_executor::task]
pub async fn imu_rate_monitor() {
    ozonide_core::tasks::rate_monitor("IMU", IMU_TOPIC.count()).await;
}
