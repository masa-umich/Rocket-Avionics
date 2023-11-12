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
  /* USER CODE BEGIN 5 */
  int channels = 1;
  float voltages[channels];
  uint16_t adc_values[channels];
  ADCinit(&hadc1, channels, adc_values);
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_12); //"DAC" Pin
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); //LED
	readAllADC(adc_values, channels, voltages);
	osDelay(750);
  }
```

This driver is only meant to work with the ADCs in DMA (Direct Memory Access) mode so that the data does not have to go through the processor. For instructions on how to do this, follow [this](https://www.digikey.com/en/maker/projects/getting-started-with-stm32-working-with-adc-and-dma/f5009db3a3ed4370acaf545a3370c30c) tutorial. (On STM32H7 Chips, the DMA setting are under "Conversion Data Mode" in the IOC file)

Note: If DMA is in continous mode, it may interupt the memory all of the time (like it's supposed to do) but this means that we cannot read that memory, so you may have to lower the polling rate for the ADCs so that we can actually read them.

You could probably fix this by designing this to work in DMA one-shot mode and then restarting DMA every time you want to read, but that would be slower.\
You could probably also add code to stop DMA for the short time we want to read the memory.\
The best solution to this would probably be to not use DMA and re-write this driver to just use normal HAL ADC functions, but I'm too far down the rabbit hole to do that right now and technically, this is faster.