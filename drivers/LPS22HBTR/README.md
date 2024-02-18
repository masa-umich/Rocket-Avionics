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
	  TickType_t xLastWakeTime;
	  const TickType_t xFrequency = 200; //Milliseconds
	  // Initialize the xLastWakeTime variable with the current time.
	  xLastWakeTime = xTaskGetTickCount();


	  float Alt;
	  float Pres;

	  BAR BAR1 = {0};
	  BAR1.hspi = &hspi1;
	  BAR1.SPI_TIMEOUT = 9999;
	  BAR1.CS_GPIO_Port = GPIOA;
	  BAR1.CS_GPIO_Pin = GPIO_PIN_8;
	  BAR1.pres_offset = 0;
	  BAR1.alt_offset = 0;

	  BAR_init(&BAR1);

	  for(;;) {
		  vTaskDelayUntil(&xLastWakeTime, xFrequency);
		  //BAR_getPres(&BAR1, &Pres);
		  BAR_getTemp(&BAR1, &Alt);
	  }
```