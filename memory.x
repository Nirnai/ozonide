/* Memory layout for STM32H743VIT6 
 * 
 * STM32H7 Multi-Domain RAM Architecture:
 * - D1 domain: AXI SRAM (512KB) - main RAM, fastest for general use
 * - D2 domain: SRAM1/2/3 (288KB) - for peripherals and DMA
 * - D3 domain: SRAM4 (64KB) - backup domain, low-power retention
 * - DTCM: (128KB) - CPU-only, zero wait state, not DMA accessible
 */

MEMORY
{
  /* Flash memory: 2MB total */
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  
  /* RAM - Default region for stack, heap, and .data/.bss
   * Using DTCM for fastest stack/interrupt performance */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
  
  /* DTCM RAM - Tightly Coupled Memory (CPU only, fastest)
   * Use for: stack, time-critical variables, ISR data
   * NOT accessible by DMA */
  DTCMRAM : ORIGIN = 0x20000000, LENGTH = 128K
  
  /* AXI SRAM (D1 domain) - Main working memory
   * Use for: heap, large buffers, general data, DMA-accessible
   * Accessible by: CPU, DMA, Ethernet, all peripherals */
  AXISRAM : ORIGIN = 0x24000000, LENGTH = 512K
  
  /* SRAM1 (D2 domain) - For D2 peripherals and DMA
   * Use for: DMA buffers for most peripherals (SPI, I2C, UART, ADC, etc.) */
  SRAM1 : ORIGIN = 0x30000000, LENGTH = 128K
  
  /* SRAM2 (D2 domain) - Additional D2 peripheral memory */
  SRAM2 : ORIGIN = 0x30020000, LENGTH = 128K
  
  /* SRAM3 (D2 domain) - Smaller D2 peripheral memory */
  SRAM3 : ORIGIN = 0x30040000, LENGTH = 32K
  
  /* SRAM4 (D3 domain) - Backup domain RAM
   * Use for: data retention in standby mode, D3 peripheral DMA (LPUART, I2C4, SPI6)
   * Retains data in low-power modes */
  SRAM4 : ORIGIN = 0x38000000, LENGTH = 64K
}


SECTIONS
{
  .camera_buffers (NOLOAD) : ALIGN(4)
  {
    *(.camera_buffers .camera_buffers.*);
    . = ALIGN(4);
  } > AXISRAM

  .sensor_buffers (NOLOAD) : ALIGN(4)
  {
    *(.sensor_buffers .sensor_buffers.*);
    . = ALIGN(4);
  } > SRAM1

  .algorithm_buffers (NOLOAD) : ALIGN(4)
  {
    *(.algorithm_buffers .algorithm_buffers.*);
    . = ALIGN(4);
  } > SRAM2

  .control (NOLOAD) : ALIGN(4)
  {
    *(.control .control.*);
    . = ALIGN(4);
  } > SRAM3

  .persistent (NOLOAD) : ALIGN(4)
  {
    *(.persistent .persistent.*);
    . = ALIGN(4);
  } > SRAM4
}
