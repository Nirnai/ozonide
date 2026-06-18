use serde::{Deserialize, Serialize};

/// Per-motor speed feedback from ESC telemetry or the simulator's actuator model.
///
/// # Source
///
/// **Hardware (DShot bidirectional):** the ESC reports electrical RPM (eRPM).
/// Convert before publishing: `Ω = eRPM / pole_pairs × 2π / 60`.
/// Use the motor's pole-pair count from its datasheet. Validate the integer
/// with the bench hover-invariant check `(G·u₀)[3] ≈ 1.0` — a wrong
/// pole-pair count introduces a quadratic error in `u₀ = Ω²`.
///
/// **SITL:** the simulator's first-order motor lag model integrates `Ω`
/// directly; the value can be read without conversion.
///
/// # Update rate
///
/// DShot telemetry arrives at ESC frame rate (typically 1–8 kHz), but the
/// state estimation task only samples it when a new frame is available.
/// The `timestamp_us` reflects when the ESC frame was received, not the
/// IMU sample time — consumers should age-gate stale frames.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ActuatorTelemetry {
    /// Timestamp when the ESC telemetry frame was received (µs since boot).
    pub timestamp_us: u64,
    /// Mechanical angular speed Ω per motor, rad/s. Order: `[FR, RL, FL, RR]`.
    pub motor_speed: [f32; 4],
}
