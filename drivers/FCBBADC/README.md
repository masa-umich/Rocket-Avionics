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

This driver is only meant to work with the ADCs in DMA (Direct Memory Access) mode so that the data does not have to go through the processor. For instructions on how to do this, follow [this](https://www.digikey.com/en/maker/projects/getting-started-with-stm32-working-with-adc-and-dma/f5009db3a3ed4370acaf545a3370c30c) tutorial. (On STM32H7 Chips, the DMA setting are under "Conversion Data Mode" in the IOC file)