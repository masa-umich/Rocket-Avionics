# Sesnor Emulation Board Firmware & Software
Author: Jack Hammerberg\
Last updated: 10/2/2025

## Application Notes
* You MUST use hardware NSS when configuring a SPI slave in STM32! The software option will not allow the MISO register to get shifted out
* If re-using the MS5611 emulator code, remember to call `emulation_init()` before your main loop, `emulation_loop(<serial uart>)` in the main loop, and setup SPI interrupts w/ HAL calls disabled and replaced with `emulation_IRQHandler(<spi bus>)`