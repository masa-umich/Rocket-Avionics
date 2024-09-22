/*
 * FCBBADC.h
 *
 *  Created on: November 7, 2023
 *      Author: jackmh
 */

#ifndef FCBBADC_H	// Begin header include protection
#define FCBBADC_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

//Gives the raw value of the ADCs (0-4095) in an array (defined in ADCinit())
int readADCRaw(ADC_HandleTypeDef* hadc1, int channels, int* adcValues, int maxDelayPerChannel);

//Gives the voltage of the ADCs (0-3.3V) in an array (defined in ADCinit())
int readADCVolt(ADC_HandleTypeDef* hadc1, int channels, float* adcValues, int maxDelayPerChannel);

//Gives the voltage and time-stamps of all ADCs (0-3.3V) in an array (defined in ADCinit())
int readADC(ADC_HandleTypeDef* hadc1, int channels, float* adcValues, int maxDelayPerChannel);

#endif    // End header include protection
