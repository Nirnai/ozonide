use crate::board::traits::*;
use stm32h7xx_hal::{pac, prelude::*};
use stm32h7xx_hal::spi::{Spi, Enabled};
use stm32h7xx_hal::gpio::{Output, PushPull, ErasedPin};


pub enum Stm32h7Spi {
    Spi1(Spi<pac::SPI1, Enabled>),
    Spi2(Spi<pac::SPI2, Enabled>),
    Spi3(Spi<pac::SPI3, Enabled>),
}

impl embedded_hal::blocking::spi::Transfer<u8> for Stm32h7Spi {
    type Error = stm32h7xx_hal::spi::Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        match self {
            Self::Spi1(s) => s.transfer(words),
            Self::Spi2(s) => s.transfer(words),
            Self::Spi3(s) => s.transfer(words),
        }
    }
}

impl embedded_hal::blocking::spi::Write<u8> for Stm32h7Spi {
    type Error = stm32h7xx_hal::spi::Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        match self {
            Self::Spi1(s) => s.write(words),
            Self::Spi2(s) => s.write(words),
            Self::Spi3(s) => s.write(words),
        }
    }
}

pub struct Stm32h743vit6 {
    spi1: Option<Spi<pac::SPI1, Enabled>>,
    spi2: Option<Spi<pac::SPI2, Enabled>>,
    spi3: Option<Spi<pac::SPI3, Enabled>>,
    pa4: Option<ErasedPin<Output<PushPull>>>,
    pb0: Option<ErasedPin<Output<PushPull>>>,
}

impl BoardInterface for Stm32h743vit6 {
    type SpiBus = Stm32h7Spi;
    type OutputPin = ErasedPin<Output<PushPull>>;

    fn init() -> Self {
        defmt::info!("Initializing WeAct STM32H743VIT6 board...");
        // Get device peripherals (required for interrupt vector table)
        let _core_peripherals= cortex_m::Peripherals::take().unwrap();
        let device_peripherals = pac::Peripherals::take().unwrap();

        // Configure power and clocks
        let power_control: stm32h7xx_hal::pwr::Pwr = device_peripherals.PWR.constrain();
        let power_configuration = power_control.vos0(&device_peripherals.SYSCFG).freeze();
        let clock_control = device_peripherals.RCC.constrain();
        let clock_configuration = clock_control
            .sys_ck(480.MHz())
            .hclk(240.MHz())
            .pll1_strategy(stm32h7xx_hal::rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(100.MHz())
            .freeze(power_configuration, &device_peripherals.SYSCFG);
        defmt::info!("Clock configuration complete:");
        defmt::info!("  System Clock (SYSCLK): {} MHz", clock_configuration.clocks.sys_ck().raw() / 1_000_000);
        defmt::info!("  AHB Clock (HCLK):      {} MHz", clock_configuration.clocks.hclk().raw() / 1_000_000);
        defmt::info!("  APB1 Clock:            {} MHz", clock_configuration.clocks.pclk1().raw() / 1_000_000);
        defmt::info!("  APB2 Clock:            {} MHz", clock_configuration.clocks.pclk2().raw() / 1_000_000);
        defmt::info!("  APB3 Clock:            {} MHz", clock_configuration.clocks.pclk3().raw() / 1_000_000);
        defmt::info!("  APB4 Clock:            {} MHz", clock_configuration.clocks.pclk4().raw() / 1_000_000);

        // Configure SPI1 (PA5=SCK, PA6=MISO, PA7=MOSI)
        let gpioa = device_peripherals.GPIOA.split(clock_configuration.peripheral.GPIOA);
        let gpiob = device_peripherals.GPIOB.split(clock_configuration.peripheral.GPIOB);
        let spi1 = device_peripherals.SPI1.spi(
            (
                gpioa.pa5.into_alternate(),
                gpioa.pa6.into_alternate(),
                gpioa.pa7.into_alternate(),
            ),
            stm32h7xx_hal::spi::MODE_0,
            20.MHz(), // Needed for 32kHz imu data reading
            clock_configuration.peripheral.SPI1,    
            &clock_configuration.clocks,
        );
        let pa4 = gpioa.pa4.into_push_pull_output().erase();
        let pb0 = gpiob.pb0.into_push_pull_output().erase();

        defmt::info!("STM32H743VIT6 initialized @ 480MHz");

        return Self {
            spi1: Some(spi1),
            spi2: None,
            spi3: None,
            pa4: Some(pa4),
            pb0: Some(pb0),
        };
    }

    fn take_spi(&mut self, name: &str) -> Result<Self::SpiBus, BoardError> {
        match name {
            "SPI1" => self.spi1.take()
                .map(Stm32h7Spi::Spi1)
                .ok_or(BoardError::PeripheralAlreadyTaken),
            "SPI2" => self.spi2.take()
                .map(Stm32h7Spi::Spi2)
                .ok_or(BoardError::PeripheralAlreadyTaken),
            "SPI3" => self.spi3.take()
                .map(Stm32h7Spi::Spi3)
                .ok_or(BoardError::PeripheralAlreadyTaken),
            _ => {
                defmt::error!("Unknown SPI bus: {}", name);
                Err(BoardError::PeripheralNotFound)
            }
        }
    }

    fn take_output_pin(&mut self, name: &str) -> Result<Self::OutputPin, BoardError> {
        match name {
            "PA4" => self.pa4.take().ok_or(BoardError::PeripheralAlreadyTaken),
            "PB0" => self.pb0.take().ok_or(BoardError::PeripheralAlreadyTaken),
            _ => Err(BoardError::PeripheralNotFound),
        }
    }
}
