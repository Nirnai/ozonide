# ozonide

Ozonide is a Rust-based flight controller stack targeting STM32H7-class MCUs.  The current focus is on: 
- Sensor drivers (e.g. IMUs, barometers, magnetometers) 
- Actuator outputs for multirotors (e.g. PWM, DShot)
- A communication layer between the flight controller and companion computers or ground stations

## Hardware Target

**Board:** WeAct STM32H743 STM32H743VIT6  
**MCU:** STM32H743VIT6 (Cortex-M7 @ 480MHz, 2MB Flash, 1MB RAM)

## Peripherals

┌─────────────────────────────────────┐
│   STM32H743 Chip                    │
│                                     │
│  ┌───────────────────────────────┐ │
│  │ ARM Cortex-M7 Core            │ │
│  │  - SYST (SysTick)             │ │
│  │  - NVIC                       │ │  ← cortex_m::Peripherals
│  │  - SCB                        │ │
│  │  - FPU, MPU, etc.             │ │
│  └───────────────────────────────┘ │
│                                     │
│  STM32-Specific Peripherals:       │
│  - GPIO (A-K)                      │
│  - Timers (1-17)                   │  ← pac::Peripherals
│  - UART, SPI, I2C                  │
│  - USB, Ethernet                   │
│  - ADC, DAC                        │
│  - PWR, RCC (power/clocks)         │
└─────────────────────────────────────┘

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
