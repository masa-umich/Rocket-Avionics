# MS5611 Driver
README Last Updated: 2/16/2025

Primary Author: Jackmh

## Overview
This is a driver for the secondary barometer, the MS5611, used to read pressure and derive altitude.

## Example usage:
```c
  MS5611 bar;
  bar.hspi = &hspi3;
  bar.SPI_TIMEOUT = 100;
  bar.CS_GPIO_Port = GPIOD; // PD2
  bar.CS_GPIO_Pin = GPIO_PIN_2;
  bar.pres_offset = 0;
  bar.alt_offset = 0;

  MS5611_PROM_t prom;
  prom.constants.C1 = 0;
  prom.constants.C2 = 0;
  prom.constants.C3 = 0;
  prom.constants.C4 = 0;
  prom.constants.C5 = 0;
  prom.constants.C6 = 0;

  MS5611_Reset(&bar);
  MS5611_readPROM(&bar, &prom);

  float pres = 0.0;
  float alt = 0.0;

  /* Infinite loop */
  for(;;) {
    MS5611_getPres(&bar, &pres, &prom, OSR_256);
    MS5611_getAlt(&bar, &alt, &prom, OSR_256); // Altitude from sea-level in feet
    osDelay(500);
  }
```