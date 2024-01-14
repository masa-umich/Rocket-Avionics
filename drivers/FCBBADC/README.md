## ADC Drivers:
* Name: FCBBADC (Flight Computer & Bay Board Analog to Digital Converter)
* Init ADCs in batch
* Read in batch
* Convert to raw in batch
* Convert to voltages in batch (float32 array)
* Driver Functions:
    * Init
    * Get Raw Values
    * Get Raw Voltages
    * Get Voltages & Timestamps

Here is some example code for how to use this driver:
```c
  	TickType_t xLastWakeTime;
	  const TickType_t xFrequency = 750; //Milliseconds
	  // Initialize the xLastWakeTime variable with the current time.
	  xLastWakeTime = xTaskGetTickCount();
	  int channels = 4;
	  float adcValues[4];

	  for(;;) {
		  vTaskDelayUntil(&xLastWakeTime, xFrequency);
		  readADCRaw(&hadc1, channels, adcValues, 500);
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //LED
	  }
```
***
This code is used to read ADCs in batch. To setup the configuration to use this code, go to the ADC1 Mode & Configuration page in CubeIDE and set "Number of Conversions" under "ADC_Regular_Conversion_Mode" to the amount of channels you want. Then set the different pins you would like to read as different inputs to that ADC. Now, open the "rank" tabs where you should find a new tab for every channel you specified. Set the channel inside each rank to the corresponding pin, keep in mind the order as the driver does not know what pin is actually being read, only that it does them in order. Do not enable "Continuous Conversion" as this will break the code.
***
There is currently a bug with this driver, for some reason the returned value of adcValues (the float32 array) is not the correct order and can be missing some channels. I have no idea why it is out of order sometimes, I believe it's a configuration issue, or probably an issue with restarting the ADC at improper times. If it's missing some channels, that may be because the error checking is enabled, where it will exit the function if ```c HAL_ADC_PollForConversion(hadc1, maxDelayPerChannel) != HAL_OK```
***
ADCinit does nothing for right now and might not be included in the final version, as it looks like the ADC will need to be restarted every time you want to sample the ADCs. 