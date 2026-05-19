//! Clock Verification: SysTick delay
//!
//! Toggles PE3 every delay_ms(1) using the HAL SysTick delay provider.
//! Expected full period: 2 ms regardless of whether SysTick is clocked at 480 or 240 MHz,
//! because the HAL configures the delay from the known clock value.
//!
//! Also logs the CYCCNT-measured period so you can compare with clock_asm_delay.
//! If both show the same scope period, clocks are consistent.
//! If asm::delay shows 4 ms but SysTick shows 2 ms, asm::delay runs at HCLK (240 MHz).
//!
//! Probe PE3 on your oscilloscope.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use panic_probe as _;
use stm32h7xx_hal::{pac, prelude::*};

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

    // SysTick delay — clocked by sys_ck per HAL configuration
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // PE3 as push-pull output (LED / test point)
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let mut pin = gpioe.pe3.into_push_pull_output();

    defmt::info!("Toggling PE3 every delay_ms(1). Expected half-period: 1 ms.");

    let mut last_cyccnt: u32 = unsafe { (*cortex_m::peripheral::DWT::PTR).cyccnt.read() };

    loop {
        pin.set_high();
        delay.delay_ms(1_u32);
        pin.set_low();
        delay.delay_ms(1_u32);

        let now: u32 = unsafe { (*cortex_m::peripheral::DWT::PTR).cyccnt.read() };
        let period_cycles = now.wrapping_sub(last_cyccnt);
        last_cyccnt = now;

        let period_us_at_480 = period_cycles / 480;
        defmt::info!(
            "period_cycles={} → {} µs (if CYCCNT@480MHz)  |  {} µs (if CYCCNT@240MHz)",
            period_cycles,
            period_us_at_480,
            period_cycles / 240,
        );
    }
}
