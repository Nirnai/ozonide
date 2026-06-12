use serde::{Deserialize, Serialize};

/// Battery voltage measurement.
///
/// Slowly varying — typical update rates are 10–100 Hz. Staleness of a few
/// hundred milliseconds is harmless for the voltage-compensation use case.
/// Never divide by `voltage` without checking that the value is live and sane
/// (> some minimum, e.g. 3.0 V/cell × cell count).
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct BatteryStatus {
    /// Timestamp of the ADC sample (µs since boot).
    pub timestamp_us: u64,
    /// Pack voltage, volts.
    pub voltage: f32,
}
