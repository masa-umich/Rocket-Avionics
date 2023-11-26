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
    for (int i = 0; i < channels; i++) {
        voltage_values[i] = (float)adc_values[i] * 3.3 / 4096.0; //Convert each ADC value to a voltage value
    }
    return 0;
}

int readADC(uint16_t adc_value[], uint8_t channel, float voltage_value) {
    voltage_value = (float)adc_value[channel] * 3.3 / 4096.0;
    return 0;
}
