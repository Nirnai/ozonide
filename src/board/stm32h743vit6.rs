use crate::board::traits::*;
use crate::config;

use stm32h7xx_hal::{interrupt, pac, prelude::*};
use stm32h7xx_hal::spi::{Spi, Enabled};
use stm32h7xx_hal::gpio::{ErasedPin, Input, Output, PushPull, ExtiPin};

pub const SYSTEM_FREQUENCY_HZ: u32 = 480_000_000;
pub const CPU_FREQUENCY_HZ: u32 = 240_000_000;

pub struct Stm32h743vit6 {
    spi1: Option<Spi<pac::SPI1, Enabled>>,
    pa4: Option<ErasedPin<Output<PushPull>>>,
    pb0: Option<ErasedPin<Input>>,
}

impl BoardInterface for Stm32h743vit6 {
    type SpiInterface = Spi<pac::SPI1, Enabled>;
    type OutputPin = ErasedPin<Output<PushPull>>;
    type InterruptPin = ErasedPin<Input>;

    fn init() -> Self {
        defmt::info!("Initializing WeAct STM32H743VIT6 board...");
        // Get device peripherals (required for interrupt vector table)
        let device_peripherals = pac::Peripherals::take().unwrap();

        // Configure power and clocks
        let power_control: stm32h7xx_hal::pwr::Pwr = device_peripherals.PWR.constrain();
        let power_configuration = power_control.vos0(&device_peripherals.SYSCFG).freeze();
        let clock_control = device_peripherals.RCC.constrain();
        // defmt::info!("CPU Frequency: {} Hz", clock_control.clocks.sysclk().to_Hz());
        let clock_configuration = clock_control
            .sys_ck(SYSTEM_FREQUENCY_HZ.Hz())
            .hclk(CPU_FREQUENCY_HZ.Hz())
            .pll1_strategy(stm32h7xx_hal::rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(100.MHz())
            .freeze(power_configuration, &device_peripherals.SYSCFG);

        // Configure SPI1 (PA5=SCK, PA6=MISO, PA7=MOSI)
        let spi_clock = match config::board::peripherals::spi1::CLOCK_SPEED {
            "20MHz" => 20.MHz(),
            "10MHz" => 10.MHz(),
            "5MHz" => 5.MHz(),
            _ => panic!("Unsupported SPI clock speed: {}", config::board::peripherals::spi1::CLOCK_SPEED),
        };
        let gpioa = device_peripherals.GPIOA.split(clock_configuration.peripheral.GPIOA);
        let spi1 = device_peripherals.SPI1.spi(
            (
                gpioa.pa5.into_alternate(),
                gpioa.pa6.into_alternate(),
                gpioa.pa7.into_alternate(),
            ),
            stm32h7xx_hal::spi::MODE_0,
            spi_clock,
            clock_configuration.peripheral.SPI1,
            &clock_configuration.clocks,
        );

        // Chip Select pin for SPI1 (PA4)
        let pa4 = gpioa.pa4.into_push_pull_output().erase();
        // Interrupt pin (PB0)
        let gpiob = device_peripherals.GPIOB.split(clock_configuration.peripheral.GPIOB);
        let mut pb0 = gpiob.pb0.into_pull_down_input();
        // Configure EXTI (External Interrupt)
        let mut syscfg = device_peripherals.SYSCFG;
        let mut exti = device_peripherals.EXTI;
        pb0.make_interrupt_source(&mut syscfg);
        pb0.trigger_on_edge(&mut exti, stm32h7xx_hal::gpio::Edge::Rising);
        pb0.enable_interrupt(&mut exti);
        // Enable EXTI0 in NVIC (Nested Vectored Interrupt Controller)
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI0);
        }

        defmt::info!("STM32H743VIT6 initialized @ 480MHz");

        return Self {
            spi1: Some(spi1),
            pa4: Some(pa4),
            pb0: Some(pb0.erase()),
        };
    }

    fn take_spi(&mut self, name: &str) -> Result<Self::SpiInterface, BoardError> {
        match name {
            "SPI1" => self.spi1.take().ok_or(BoardError::PeripheralAlreadyTaken),
            _ => {
                defmt::error!("Unknown SPI bus: {}", name);
                Err(BoardError::PeripheralNotFound)
            }
        }
    }

    fn take_output_pin(&mut self, name: &str) -> Result<Self::OutputPin, BoardError> {
        match name {
            "PA4" => self.pa4.take().ok_or(BoardError::PeripheralAlreadyTaken),
            _ => Err(BoardError::PeripheralNotFound),
        }
    }

    fn take_interrupt_pin(&mut self, name: &str) -> Result<Self::InterruptPin, BoardError> {
        match name {
            "PB0" => self.pb0.take().ok_or(BoardError::PeripheralAlreadyTaken),
            _ => Err(BoardError::PeripheralNotFound),
        }
    }

    #[inline(always)]
    fn clear_exti_flag<const N: u8>() {
        unsafe { (*pac::EXTI::ptr()).cpupr1.write(|w| w.bits(1 << N)); }
    }
}

#[interrupt]
fn EXTI0() {
    Stm32h743vit6::clear_exti_flag::<0>();
    crate::vehicle::Vehicle::read_imu();
}
