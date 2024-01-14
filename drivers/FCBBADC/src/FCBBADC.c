/*
 * FCBBADC.h
 *
 *  Created on: November 7, 2023
 *      Author: jackh
 *
*/

#include "../inc/FCBBADC.h"

//Initializes the ADCs and gets the array of channels to read
int ADCinit(ADC_HandleTypeDef* hadc1) {
    // Start the ADC conversion
	if (HAL_ADC_Start(hadc1) != HAL_OK) {
        //Error_Handler();
        return 1;
    }
    return 0;
}

//Gives the raw value of the ADCs (0-4095) in an array that contains all channels
int readADCRaw(ADC_HandleTypeDef* hadc1, int channels, float* adcValues, int maxDelayPerChannel) {
    HAL_ADC_Start(hadc1);
    // Loop through the channels
    for (int i = 0; i < channels; i++) {
        // Wait for the ADC conversion to complete

    	/*
    	if (HAL_ADC_PollForConversion(hadc1, maxDelayPerChannel) != HAL_OK) {
            //Error_Handler();
            return 1;
        }
		*/
    	HAL_ADC_PollForConversion(hadc1, maxDelayPerChannel);
        // Get the ADC value
        // This will also increment the ADC channel for the next time we call HAL_ADC_PollForConversion()
        uint32_t adcValue = HAL_ADC_GetValue(hadc1);

        // Store the ADC value in the channels array
        adcValues[i] = (float)adcValue;
    }
    HAL_ADC_Stop(hadc1);
    /*
    if (HAL_ADC_Stop(hadc1) != HAL_OK) {
        //Error_Handler();
        return 1;
    }
	*/
    return 0;
}

//Gives the voltage of the ADCs (0-3.3V) in an array that contains all channels
int readADCVolt(ADC_HandleTypeDef* hadc1, int channels, float* adcValues, int maxDelayPerChannel) {
    return 0;
}

//Gives the voltage and timestamps of all ADCs (0-3.3V) in an array that contains all channels
int readADC(ADC_HandleTypeDef* hadc1, int channels, float* adcValues, int maxDelayPerChannel) {
    return 0;
}
