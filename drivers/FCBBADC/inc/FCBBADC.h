/*
 * FCBBADC.h
 *
 *  Created on: November 7, 2023
 *      Author: jackh
 */

#ifndef FCBBADC	// Begin header include protection
#define FCBBADC

#include "stm32h7xx_hal.h"

//Starts DMA for all ADCs
void ADCinit(ADC_HandleTypeDef hadc1, DMA_HandleTypeDef hdma_adc1, uint8_t channels);

//Read all adcs and return the voltage values in a float array
float* readAllADC(ADC_HandleTypeDef hadc1, DMA_HandleTypeDef hdma_adc1);

//Read a single adc and return the voltage value
float readADC(uint8_t adcpin);