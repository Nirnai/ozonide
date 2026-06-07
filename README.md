# ozonide

## Running & Testing

The workspace default build target is `thumbv7em-none-eabihf` (embedded). SITL, simulator, and tests must be run with an explicit host target.

### Tests

```bash
# All testable crates (ozonide-core, sitl, simulator)
cargo test --target x86_64-unknown-linux-gnu

# Single crate
cargo test -p ozonide-core --target x86_64-unknown-linux-gnu
cargo test -p simulator    --target x86_64-unknown-linux-gnu
cargo test -p sitl         --target x86_64-unknown-linux-gnu
```

### Simulator (physics engine + WebSocket frontend)

```bash
cargo run -p simulator --target x86_64-unknown-linux-gnu
```

### SITL (software-in-the-loop flight controller)

Run together with the simulator — simulator on one terminal, SITL on another:

```bash
# Terminal 1
cargo run -p simulator --target x86_64-unknown-linux-gnu

# Terminal 2
cargo run -p sitl --target x86_64-unknown-linux-gnu
```

### Firmware (STM32H743, requires probe-rs and a connected debugger)

```bash
cargo run -p firmware
# or explicitly:
cargo run -p firmware --target thumbv7em-none-eabihf
```

Flash without running:
```bash
cargo build -p firmware --release
probe-rs download --chip STM32H743VITx target/thumbv7em-none-eabihf/release/firmware
```

Ozonide is a Rust-based flight controller stack targeting STM32H7-class MCUs.  The current focus is on: 
- Sensor drivers (e.g. IMUs, barometers, magnetometers) 
- Actuator outputs for multirotors (e.g. PWM, DShot)
- A communication layer between the flight controller and companion computers or ground stations

## Hardware Target

**Board:** WeAct STM32H743 STM32H743VIT6  
**MCU:** STM32H743VIT6 (Cortex-M7 @ 480MHz, 2MB Flash, 1MB RAM)

## Peripherals

```
┌──────────────────────────────────────────────────────────┐
│   STM32H743 Chip                                         │
│                                                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │ ARM Cortex-M7 Core                                 │  │
│  │  - SYST (SysTick)  ← managed by Embassy internally │  │
│  │  - NVIC            ← managed by Embassy internally │  │
│  │  - SCB                                             │  │
│  │  - DWT (cycle counter)                             │  │
│  │  - FPU, MPU                                        │  │
│  └────────────────────────────────────────────────────┘  │
│                                                          │
│  STM32-Specific Peripherals:  ← embassy_stm32::init()   │
│  - GPIO (A-K)                                            │
│  - Timers (1-17)                                         │
│  - UART, SPI, I2C                                        │
│  - USB, Ethernet                                         │
│  - ADC, DAC                                              │
│  - PWR, RCC (power/clocks)                               │
└──────────────────────────────────────────────────────────┘
```

## Memory Architecture

The STM32H7 features a multi-domain memory architecture optimized for high-performance embedded applications. Memory regions are mapped to specific use cases:

### Memory Region Summary

| Region | Size | Address | Application Use | DMA | Domain |
|--------|------|---------|-----------------|-----|--------|
| **DTCM** | 128KB | 0x2000_0000 | Stack, heap, ISR data | ❌ | CPU-only |
| **AXI SRAM** | 512KB | 0x2400_0000 | `.camera_buffers` - Camera, VIO | ✅ | D1 |
| **SRAM1** | 128KB | 0x3000_0000 | `.sensor_buffers` - IMU, SD card | ✅ | D2 |
| **SRAM2** | 128KB | 0x3002_0000 | `.algorithm_buffers` - MPC, estimation | ✅ | D2 |
| **SRAM3** | 32KB | 0x3004_0000 | `.control` - ESC/PWM state | ✅ | D2 |
| **SRAM4** | 64KB | 0x3800_0000 | `.persistent` - Calibration | ✅ | D3 |

### Using Memory Sections in Code

Place variables in specific memory regions using linker sections:

```rust
// Camera frame buffer in AXI SRAM (512KB available)
#[link_section = ".camera_buffers"]
static mut CAMERA_FRAME: [u8; 153600] = [0; 153600];  // 320x240 RGB565

// IMU DMA buffer in SRAM1 (D2 domain, required for SPI DMA)
#[link_section = ".sensor_buffers"]
static mut IMU_DMA_BUFFER: [u8; 1024] = [0; 1024];

// MPC workspace in SRAM2
#[link_section = ".algorithm_buffers"]
static mut MPC_WORKSPACE: [f32; 10000] = [0.0; 10000];

// ESC output state in SRAM3
#[link_section = ".control"]
static mut ESC_STATE: [u16; 4] = [0; 4];

// Calibration data in SRAM4 (survives reboots and low-power modes)
#[link_section = ".persistent"]
static mut IMU_CALIBRATION: CalibData = CalibData::default();
```

## Timers

| Timer | Type | Bits | Channels | Bus | Assignment |
|-------|------|------|----------|-----|------------|
| **TIM1** | Advanced | 16 | 4 + complementary + break | APB2 | ESC motor outputs |
| **TIM2** | General-purpose | 32 | 4 | APB1 | Reserve — encoder / RC input |
| **TIM3** | General-purpose | 16 | 4 | APB1 | Camera gimbal servos |
| **TIM4** | General-purpose | 16 | 4 | APB1 | Free |
| **TIM5** | General-purpose | 32 | 4 | APB1 | Embassy time driver |
| **TIM6** | Basic | 16 | — | APB1 | Free (DAC trigger if needed) |
| **TIM7** | Basic | 16 | — | APB1 | Free (DAC trigger if needed) |
| **TIM8** | Advanced | 16 | 4 + complementary + break | APB2 | Reserve — 8-motor / octocopter |
| **TIM12** | General-purpose | 16 | 2 | APB1 | Free |
| **TIM13** | General-purpose | 16 | 1 | APB1 | Buzzer / LED PWM |
| **TIM14** | General-purpose | 16 | 1 | APB1 | Free |
| **TIM15** | General-purpose | 16 | 2 (CH1 complementary) | APB2 | Free |
| **TIM16** | General-purpose | 16 | 1 + complementary | APB2 | Free |
| **TIM17** | General-purpose | 16 | 1 + complementary | APB2 | Free |

## IMU

```
ICM42688 Module    →    STM32H743VIT6
─────────────────────────────────────
VDD                →    3.3V
GND                →    GND  
VDDIO (if separate)→    3.3V
SCK                →    PA5 (SPI1_SCK)
MOSI (SDI)         →    PA7 (SPI1_MOSI)
MISO (SDO)         →    PA6 (SPI1_MISO)
CS (nCS)           →    PA4 (GPIO)
INT1 (optional)    →    PB0 (GPIO/EXTI)
INT2 (optional)    →    PB1 (GPIO/EXTI)
```