/*
 * FCBBADC.h
 *
 *  Created on: November 7, 2023
 *      Author: jackh
 */

#ifndef FCBBADC_H	// Begin header include protection
#define FCBBADC_H

#include "stm32h7xx_hal.h"

//Needed callback function for DMA
extern volatile uint8_t dma_transfer_complete;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1);

//Starts DMA for all ADCs
int ADCinit(ADC_HandleTypeDef* hadc1, uint8_t channels, uint16_t* adc_values);

//Read all adcs and return the voltage values in a float array
int readAllADC(uint16_t* adc_value, uint8_t channels, float* voltage_values);

//Read a single adc and return the voltage value
int readADC(uint16_t adc_value[], uint8_t channel, float voltage_value);

#endif    // End header include protection
