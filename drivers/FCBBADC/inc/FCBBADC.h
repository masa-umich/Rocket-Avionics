/*
 * FCBBADC.h
 *
 *  Created on: November 7, 2023
 *      Author: jackh
 */

#ifndef FCBBADC_H	// Begin header include protection
#define FCBBADC_H

#include "stm32h7xx_hal.h"

//Initializes the ADCs and gets the array of channels to read
int ADCinit(ADC_HandleTypeDef* hadc1);

//Gives the raw value of the ADCs (0-4095) in an array (defined in ADCinit())
int readADCRaw(ADC_HandleTypeDef* hadc1, int channels, float* adcValues, int maxDelayPerChannel);

//Gives the voltage of the ADCs (0-3.3V) in an array (defined in ADCinit())
int readADCVolt(ADC_HandleTypeDef* hadc1, int channels, float* adcValues, int maxDelayPerChannel);

//Gives the voltage and timestamps of all ADCs (0-3.3V) in an array (defined in ADCinit())
int readADC(ADC_HandleTypeDef* hadc1, int channels, float* adcValues, int maxDelayPerChannel);

#endif    // End header include protection
