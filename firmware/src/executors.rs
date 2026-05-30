use embassy_executor::{Executor, SendSpawner, Spawner};
use embassy_stm32::interrupt::{self, InterruptExt, Priority};
use static_cell::StaticCell;

static HIGH_PRIORITY_EXECUTOR: embassy_executor::InterruptExecutor = embassy_executor::InterruptExecutor::new();
static LOW_PRIORITY_EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[cortex_m_rt::interrupt]
unsafe fn TIM7() {
    HIGH_PRIORITY_EXECUTOR.on_interrupt()
}

pub fn high_priority_executor() -> SendSpawner {
    interrupt::TIM7.set_priority(Priority::P6);
    return HIGH_PRIORITY_EXECUTOR.start(interrupt::TIM7)
}

pub fn low_priority_executor<F: FnOnce(Spawner)>(f: F) -> ! {
    let executor = LOW_PRIORITY_EXECUTOR.init(Executor::new());
    executor.run(f)
}
