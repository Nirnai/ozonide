mod stm32h743vit6;

pub use stm32h743vit6::Stm32h743vit6;

pub trait Board {
    type CorePeripherals;
    type DevicePeripherals;

    fn init() -> Self;
    fn core_peripherals(&self) -> &Self::CorePeripherals;
    fn device_peripherals(&self) -> &Self::DevicePeripherals;
}