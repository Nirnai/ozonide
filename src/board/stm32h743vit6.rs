use super::Board;

use stm32h7xx_hal::{pac, prelude::*};

pub struct Stm32h743vit6 {
    core_peripherals: cortex_m::Peripherals,
    device_peripherals: stm32h7xx_hal::rcc::Ccdr,
}

impl Board for Stm32h743vit6 {
    type CorePeripherals = cortex_m::Peripherals;
    type DevicePeripherals = stm32h7xx_hal::rcc::Ccdr;

    fn init() -> Self {
        defmt::info!("Initializing WeAct STM32H743VIT6 board...");
        // Get device peripherals (required for interrupt vector table)
        let core_peripherals= cortex_m::Peripherals::take().unwrap();
        let device_peripherals = pac::Peripherals::take().unwrap();

        // Configure power and clocks
        let power_control: stm32h7xx_hal::pwr::Pwr = device_peripherals.PWR.constrain();
        let power_configuration = power_control.vos0(&device_peripherals.SYSCFG).freeze();
        let clock_control = device_peripherals.RCC.constrain();
        let clock_configuration = clock_control
            .sys_ck(480.MHz())
            .hclk(240.MHz())
            .pll1_strategy(stm32h7xx_hal::rcc::PllConfigStrategy::Iterative)
            .freeze(power_configuration, &device_peripherals.SYSCFG);
        defmt::info!("Clock configuration complete:");
        defmt::info!("  System Clock (SYSCLK): {} MHz", clock_configuration.clocks.sys_ck().raw() / 1_000_000);
        defmt::info!("  AHB Clock (HCLK):      {} MHz", clock_configuration.clocks.hclk().raw() / 1_000_000);
        defmt::info!("  APB1 Clock:            {} MHz", clock_configuration.clocks.pclk1().raw() / 1_000_000);
        defmt::info!("  APB2 Clock:            {} MHz", clock_configuration.clocks.pclk2().raw() / 1_000_000);
        defmt::info!("  APB3 Clock:            {} MHz", clock_configuration.clocks.pclk3().raw() / 1_000_000);
        defmt::info!("  APB4 Clock:            {} MHz", clock_configuration.clocks.pclk4().raw() / 1_000_000);
        defmt::info!("Hardware initialized - STM32H743 @ 480MHz");
        return Self {
            core_peripherals,
            device_peripherals: clock_configuration,
        };
    }

    fn core_peripherals(&self) -> &Self::CorePeripherals {
        return &self.core_peripherals;
    }

    fn device_peripherals(&self) -> &Self::DevicePeripherals {
        return &self.device_peripherals;
    }

}

impl Stm32h743vit6 {
    pub fn init() -> Self {
        <Self as Board>::init()
    }
}
