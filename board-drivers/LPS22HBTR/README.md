# LPS22HBTR 
* [LPS22HBTR Datasheet](https://www.st.com/content/ccc/resource/technical/document/datasheet/bf/c1/4f/23/61/17/44/8a/DM00140895.pdf/files/DM00140895.pdf/jcr:content/translations/en.DM00140895.pdf)
* Type: Barometer/Altimeter 
* Interface: SPI1
* Driver Requirements:
    * Init
    * Send Command
    * Read Register
    * Write Register
    * Get Altitude
    * Get Pressure

Example Usage:

```C
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100; //Milliseconds
  xLastWakeTime = xTaskGetTickCount();
  float Pres = 0.0;
  float Temp = 0.0;

  BAR BAR1 = {0};
  BAR1.hspi = &hspi6;
  BAR1.SPI_TIMEOUT = 1000;
  BAR1.CS_GPIO_Port = GPIOA;
  BAR1.CS_GPIO_Pin = GPIO_PIN_4;
  BAR1.pres_offset = 0;
  BAR1.alt_offset = 0;

  if (BAR_init(&BAR1)) {
	  Error_Handler(); //We have not read the who am I register, so something is probably wrong
  }

  for(;;) {
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
      BAR_getPres(&BAR1, &Pres);
      BAR_getTemp(&BAR1, &Temp);
  }
```