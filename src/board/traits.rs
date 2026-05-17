
#[derive(Debug)]
pub enum BoardError {
    PeripheralNotFound,
    PeripheralAlreadyTaken,
}

pub trait BoardInterface {
    type SpiBus: embedded_hal::blocking::spi::Transfer<u8>
    + embedded_hal::blocking::spi::Write<u8>;

    type OutputPin: embedded_hal::digital::v2::OutputPin;

    fn init() -> Self;
    fn take_spi(&mut self, name: &str) -> Result<Self::SpiBus, BoardError>;
    fn take_output_pin(&mut self, name: &str) -> Result<Self::OutputPin, BoardError>;
}