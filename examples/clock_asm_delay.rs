//! Clock Verification: asm::delay
//!
//! Toggles PE3 every asm::delay(480_000) cycles.
//! If asm::delay runs at 480 MHz (sys_ck), the half-period is 1 ms → 2 ms full period.
//! If asm::delay runs at 240 MHz (HCLK),  the half-period is 2 ms → 4 ms full period.
//!
//! Also logs the CYCCNT-measured period over RTT so you can cross-check scope vs software.
//! CYCCNT assumed to run at sys_ck (480 MHz) — that is what this test validates indirectly.
//!
//! Probe PE3 on your oscilloscope.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal::{pac, prelude::*};

const DELAY_CYCLES: u32 = 480_000; // "1 ms" at 480 MHz

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // VOS0 + 480 MHz sys_ck, 240 MHz HCLK — same as main application
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0(&dp.SYSCFG).freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(480.MHz())
        .hclk(240.MHz())
        .pll1_strategy(stm32h7xx_hal::rcc::PllConfigStrategy::Iterative)
        .freeze(pwrcfg, &dp.SYSCFG);

    defmt::info!(
        "sys_ck={} MHz  hclk={} MHz",
        ccdr.clocks.sys_ck().to_MHz(),
        ccdr.clocks.hclk().to_MHz(),
    );

    // Enable DWT cycle counter
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    // PE3 as push-pull output (LED / test point)
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let mut pin = gpioe.pe3.into_push_pull_output();

    defmt::info!(
        "Toggling PE3 every asm::delay({}) cycles. Expected half-period: 1 ms at 480 MHz.",
        DELAY_CYCLES
    );

    let mut last_cyccnt: u32 = unsafe { (*cortex_m::peripheral::DWT::PTR).cyccnt.read() };

    loop {
        pin.set_high();
        cortex_m::asm::delay(DELAY_CYCLES);
        pin.set_low();
        cortex_m::asm::delay(DELAY_CYCLES);

        // Measure full period in CYCCNT ticks (wrapping is fine for relative diff)
        let now: u32 = unsafe { (*cortex_m::peripheral::DWT::PTR).cyccnt.read() };
        let period_cycles = now.wrapping_sub(last_cyccnt);
        last_cyccnt = now;

        // If CYCCNT runs at 480 MHz: period_us = period_cycles / 480
        let period_us_at_480 = period_cycles / 480;
        defmt::info!(
            "period_cycles={} → {} µs (if CYCCNT@480MHz)  |  {} µs (if CYCCNT@240MHz)",
            period_cycles,
            period_us_at_480,
            period_cycles / 240,
        );
    }
}
