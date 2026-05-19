//! System timing primitives for STM32H743VIT6.
//!
//! # Clock architecture
//!
//! The STM32H7 has two primary CPU-visible clocks:
//!
//! - `sys_ck` (480 MHz) — the CPU core clock (FCLK). Feeds the DWT cycle counter (CYCCNT)
//!   and the instruction fetch pipeline.
//! - `HCLK`  (240 MHz) — the AHB bus clock. Feeds peripherals, SRAM, and Flash wait states.
//!
//! # Experimental verification (2026-05-19)
//!
//! Two oscilloscope experiments were run to determine which clock drives CYCCNT and
//! `cortex_m::asm::delay`, because the wrong assumption had caused the IMU ISR rate
//! measurement to read ~504 Hz instead of the true 1 kHz.
//!
//! ## Experiment 1 — SysTick (`clock_systick` example)
//!
//! The HAL SysTick delay provider was used to toggle PE3 every `delay_ms(1)`.
//! The HAL configures SysTick from the known `sys_ck` value, so the output is
//! a reliable reference independent of which physical clock CYCCNT uses.
//!
//! | Source       | Value     |
//! |--------------|-----------|
//! | Scope        | 2.125 ms  |
//! | CYCCNT / 480 | 2147 µs ✓ |
//! | CYCCNT / 240 | 4294 µs ✗ |
//!
//! **Finding: CYCCNT is clocked by sys_ck = 480 MHz.**
//!
//! ## Experiment 2 — `asm::delay` (`clock_asm_delay` example)
//!
//! `asm::delay(480_000)` was used to toggle PE3. At 480 MHz this should produce a
//! 1 ms half-period (2 ms full period). The scope measured 1.110 ms full period instead.
//!
//! | Source       | Value     |
//! |--------------|-----------|
//! | Scope        | 1.110 ms  |
//! | CYCCNT / 480 | 1132 µs ✓ |
//! | CYCCNT / 240 | 2265 µs ✗ |
//!
//! **Finding: `asm::delay(N)` consumes approximately N/2 actual clock cycles.**
//! The Cortex-M7 dual-issue pipeline fuses the `subs r0, #1` + `bne` loop body
//! into a single pipeline slot, so the loop runs at roughly twice the instruction
//! throughput. `asm::delay` is therefore unsuitable for precise timing and must
//! not be used where accuracy matters.
//!
//! # Implementation rationale
//!
//! - [`SystemTime::now`] reads CYCCNT and divides by 480 (sys_ck in MHz).
//! - [`SystemTime::delay_us`] busy-waits by spinning on CYCCNT until the required
//!   number of cycles have elapsed. This is immune to the dual-issue effect because
//!   it measures elapsed cycles directly rather than counting loop iterations.
//! - All timestamps are `u32` and wrap every ~8.9 s (2³² cycles / 480 MHz).
//!   Use [`u32::wrapping_sub`] for all elapsed-time calculations.

use crate::board;

const CYCLES_PER_US: u32 = board::SYSTEM_FREQUENCY_HZ / 1_000_000; // 480

#[inline(always)]
fn read_cyccnt() -> u32 {
    unsafe { (*cortex_m::peripheral::DWT::PTR).cyccnt.read() }
}

/// Microsecond-resolution system clock backed by the DWT cycle counter.
///
/// CYCCNT must be enabled before use — call `DWT::enable_cycle_counter()` once
/// at startup (already done in `main`).
pub struct SystemTime;

impl SystemTime {
    pub fn init() {
        let mut core_peripherals = cortex_m::Peripherals::take().unwrap();
        core_peripherals.DCB.enable_trace();
        core_peripherals.DWT.enable_cycle_counter();
    }

    #[inline(always)]
    pub fn now() -> u32 {
        read_cyccnt() / CYCLES_PER_US
    }

    #[inline(always)]
    pub fn delay(us: u32) {
        let start = read_cyccnt();
        let ticks = us * CYCLES_PER_US;
        while read_cyccnt().wrapping_sub(start) < ticks {}
    }

    #[inline(always)]
    pub fn elapsed(since: u32) -> u32 {
        Self::now().wrapping_sub(since)
    }
}
