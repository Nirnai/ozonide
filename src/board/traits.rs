use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

#[derive(Debug)]
pub enum BoardError {
    PeripheralNotFound,
    PeripheralAlreadyTaken,
}

pub trait BoardInterface {
    type SpiInterface: Transfer<u8> + Write<u8>;
    type OutputPin: OutputPin;
    type InterruptPin: InputPin;

    fn init() -> Self;
    fn take_spi(&mut self, name: &str) -> Result<Self::SpiInterface, BoardError>;
    fn take_output_pin(&mut self, name: &str) -> Result<Self::OutputPin, BoardError>;
    fn take_interrupt_pin(&mut self, name: &str) -> Result<Self::InterruptPin, BoardError>;
}