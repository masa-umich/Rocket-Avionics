# LSM6DSO32XTR
* [LSM6DSO32XTR Datasheet](https://www.st.com/resource/en/datasheet/lsm6dso32x.pdf)
* Type: IMU
* Interface: SPI1
* Driver Requirements:
    * Init
    * Send Command
    * Read Register
    * Write Register
    * Get Accelleration
    * Get Angular Rate
    * Get Temperature

***
I'll make this better later, but here's some example code:
```c
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1; //Milliseconds
	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	float temp;

	AngRate G_readings = {0};
	G_readings.G_x = 0;
	G_readings.G_y = 0;
	G_readings.G_z = 0;

	Accel XL_readings = {0};
	XL_readings.XL_x = 0;
	XL_readings.XL_y = 0;
	XL_readings.XL_z = 0;

	IMU IMU1;
	IMU1.hspi = &hspi1;
	IMU1.SPI_TIMEOUT = 9999;
	IMU1.CS_GPIO_Port = GPIOA;
	IMU1.CS_GPIO_Pin = 4;
	IMU1.XL_x_offset = 0;
	IMU1.XL_y_offset = 0;
	IMU1.XL_z_offset = 0;
	IMU1.G_x_offset = 0;
	IMU1.G_y_offset = 0;
	IMU1.G_z_offset = 0;

	IMU_init(&IMU1);
    
	for(;;) {
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
	  IMU_getAccel(&IMU1, &XL_readings);
	  IMU_getTemp(&IMU1, &temp);
	  IMU_getAngRate(&IMU1, &G_readings);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED
	}
```