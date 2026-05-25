use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::rcc::{Pll, PllDiv};
use embassy_stm32::spi::Spi;
use embassy_stm32::{bind_interrupts, exti, interrupt, spi, Config};

bind_interrupts!(struct Irqs {
    EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
});

pub struct Board {
    pub imu_spi: Spi<'static, Blocking, spi::mode::Master>,
    pub imu_chip_select: Output<'static>,
    pub imu_data_ready: ExtiInput<'static, Async>,
}

impl Board {
    pub fn init() -> Self {
        let p = embassy_stm32::init(Self::clock_config());
        let mut spi_config = spi::Config::default();
        spi_config.frequency = embassy_stm32::time::mhz(20);
        Self {
            imu_spi: Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi_config),
            imu_chip_select: Output::new(
                p.PA4,
                embassy_stm32::gpio::Level::High,
                embassy_stm32::gpio::Speed::High,
            ),
            imu_data_ready: ExtiInput::new(p.PB0, p.EXTI0, embassy_stm32::gpio::Pull::Down, Irqs),
        }
    }

    fn clock_config() -> Config {
        let mut config = Config::default();
        config.rcc.voltage_scale = embassy_stm32::rcc::VoltageScale::Scale0;
        config.rcc.pll1 = Some(Pll {
            source: embassy_stm32::rcc::PllSource::Hsi,
            prediv: embassy_stm32::rcc::PllPreDiv::Div4,
            mul: embassy_stm32::rcc::PllMul::Mul60,
            fracn: None,
            divp: Some(PllDiv::Div2),
            divq: Some(PllDiv::Div10),
            divr: None,
        });
        config.rcc.sys = embassy_stm32::rcc::Sysclk::Pll1P; // 480 MHz
        config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::Div2;  // HCLK  = 240 MHz
        config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::Div2; // APB1  = 120 MHz
        config.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::Div2; // APB2  = 120 MHz
        config.rcc.apb3_pre = embassy_stm32::rcc::APBPrescaler::Div2; // APB3  = 120 MHz
        config.rcc.apb4_pre = embassy_stm32::rcc::APBPrescaler::Div2; // APB4  = 120 MHz
        config
    }
}
