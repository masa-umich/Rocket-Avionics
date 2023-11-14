/*
 * FCBBADC.h
 *
 *  Created on: November 7, 2023
 *      Author: jackh
 *
*/

//Follow these instructions for how to setup DMA for the ADCs
//https://www.digikey.com/en/maker/projects/getting-started-with-stm32-working-with-adc-and-dma/f5009db3a3ed4370acaf545a3370c30c

#include "../inc/FCBBADC.h"

volatile uint8_t dma_transfer_complete = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1) {
    dma_transfer_complete = 1;
}

//Initialize the ADCs and set to DMA mode.
//Channels is the amount of analog pins that will be read
//The Bay Boards will have around 40
//This determines the size of the buffer
//Returns the ADC value buffer
int ADCinit(ADC_HandleTypeDef* hadc1, uint8_t channels, uint16_t* adc_values) {
    HAL_ADC_Start_DMA(hadc1, (uint32_t*)adc_values, (uint32_t)channels);
    return 0;
}

int readAllADC(uint16_t* adc_values, uint8_t channels, float* voltage_values) {
    //Note: When DMA is set to circular mode, it usually writes the values too fast and this function won't be able to read the memory.
    //To fix this, we need to interrupt the DMA while we want to read the values.
    //We cannot do this by simply restarting DMA though, as that would add latency and kind of defeat the purpose of DMA.
    //Instead we need to use the interrupt handlers/callback functions to set a flag when the DMA is done writing to memory.
    //Then we can read the values and hope that we can do that fast enough before the DMA starts writing again.
    while (dma_transfer_complete == 0) {
        //Wait for DMA to finish writing to memory
    }
    for (int i = 0; i < channels; i++) {
        voltage_values[i] = (float)adc_values[i] * 3.3 / 4096.0; //Convert each ADC value to a voltage value
    }
    return 0;
}

int readADC(uint16_t adc_value[], uint8_t channel, float voltage_value) {
    while (dma_transfer_complete == 0) {
        //Wait for DMA to finish writing to memory
    }
    voltage_value = (float)adc_value[channel] * 3.3 / 4096.0;
    return 0;
}
