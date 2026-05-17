// //! Electronic Speed Controller (ESC) drivers
// //!
// //! Provides interfaces for controlling brushless motors via:
// //! - Standard PWM (1000-2000µs)
// //! - DShot digital protocol

// use crate::types::ControlOutput;

// /// Common interface for ESC control
// pub trait EscController {
//     type Error;

//     /// Initialize ESCs with calibration sequence if needed
//     fn init(&mut self) -> Result<(), Self::Error>;

//     /// Set motor outputs (0-1000 range, will be mapped to protocol)
//     fn set_motors(&mut self, output: &ControlOutput) -> Result<(), Self::Error>;

//     /// Arm the ESCs (safety)
//     fn arm(&mut self) -> Result<(), Self::Error>;

//     /// Disarm the ESCs (motors off)
//     fn disarm(&mut self) -> Result<(), Self::Error>;

//     /// Check if ESCs are armed
//     fn is_armed(&self) -> bool;
// }

// /// PWM-based ESC driver using timers
// pub struct PwmEsc {
//     // TODO: Timer channels, armed state
// }

// /// DShot protocol ESC driver
// pub struct DshotEsc {
//     // TODO: DMA + timer configuration, telemetry
// }

// #[derive(Debug, Clone, Copy)]
// pub enum DshotSpeed {
//     Dshot150,
//     Dshot300,
//     Dshot600,
//     Dshot1200,
// }

// // TODO: Implement EscController for PwmEsc and DshotEsc
