/*
 * FCBBADC.h
 *
 *  Created on: November 7, 2023
 *      Author: jackh
 *
*/

//Follow these instructions for how to setup DMA for the ADCs
//https://www.digikey.com/en/maker/projects/getting-started-with-stm32-working-with-adc-and-dma/f5009db3a3ed4370acaf545a3370c30c

#include "../inc/W25N01GV.h"

uint16_t adc_value; //Buffer for DMA to store ADC values in

//Initialize the ADCs and set to DMA mode.
//Channels is the amount of analog pins that will be read
//The Bay Boards will have around 40
//This determines the size of the buffer
void ADCinit(ADC_HandleTypeDef hadc1, DMA_HandleTypeDef hdma_adc1, uint8_t channels) {
    uint16_t adcbuffer_size = channels * 2; //2 bytes per channel (A maximum value of 4096 for 12 bit ADCs)
    HAL_ADC_Start_DMA(&hadc1, &adc_value, &adcbuffer_size);
}

//Read all adcs and return the voltage values in a float array
float* readAllADC(ADC_HandleTypeDef hadc1, DMA_HandleTypeDef hdma_adc1) {
//Read from the DMA ADC buffer and store to an array, with the index being in the same order as the pins.
//TODO: this lmao
}

//Read a single adc and return the voltage value
float readADC(uint8_t adc);