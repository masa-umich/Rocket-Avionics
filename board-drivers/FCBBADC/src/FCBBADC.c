/*
 * FCBBADC.h
 *
 *  Created on: November 7, 2023
 *      Author: jackmh
 *
 */

#include "../inc/FCBBADC.h"

// Needed for readADC() as it also returns a timestamp
extern uint64_t getTimestamp(void);

// Internal function to get the ADC resolution, stupid that there isn't a HAL function for this
static int getADCResolution(ADC_HandleTypeDef* hadc1) {
  int resolution;
  switch (hadc1->Init.Resolution) {
    case ADC_RESOLUTION_16B:
      resolution = 65535;
      break;
    case ADC_RESOLUTION_14B:
      resolution = 16383;
      break;
    case ADC_RESOLUTION_12B:
      resolution = 4095;
      break;
    case ADC_RESOLUTION_10B:
      resolution = 1023;
      break;
    case ADC_RESOLUTION_8B:
      resolution = 255;
      break;
    default:
      resolution = 1;  // Error case
      break;
  }
  return resolution;
}

// Gives the raw value of the ADCs (0-4095) in an array that contains all
// channels
int readADCRaw(ADC_HandleTypeDef* hadc1, int channels, int* adcValues,
               int maxDelayPerChannel) {
  // This function relies on HAL switching to the next ADC rank, but this causes
  // issues with RTOS threads To fix this we make this a "critical task" so that
  // it's not interrupted. Additionally, "Sampling time" should be set no lower
  // than 64.5 per channel if you haven't already
  taskENTER_CRITICAL();

  // Start the ADC and return if not successful
  if (HAL_ADC_Start(hadc1) != HAL_OK) {
    return 1;
  }

  // Loop through the channels
  for (int i = 0; i < channels; i++) {
    // Wait for the ADC conversion to complete
    HAL_StatusTypeDef debugStatus;
    debugStatus = HAL_ADC_PollForConversion(hadc1, maxDelayPerChannel);
    if (debugStatus != HAL_OK) {
      return 1;
    }
    // Get the ADC value
    // This will also increment the ADC channel for the next time we call
    // HAL_ADC_PollForConversion()
    uint32_t adcValue = HAL_ADC_GetValue(hadc1);

    // Store the ADC value in the channels array
    adcValues[i] = (int)adcValue;
  }
  // Stop the ADC and return if not successful
  if (HAL_ADC_Stop(hadc1) != HAL_OK) {
    return 1;
  }
  // Exit critical task
  taskEXIT_CRITICAL();
  return 0;
}

// Gives the voltage of the ADCs (0-3.3V) in an array that contains all channels
int readADCVolt(ADC_HandleTypeDef* hadc1, int channels, float* adcValues,
                int maxDelayPerChannel) {
  // This function relies on HAL switching to the next ADC rank, but this causes
  // issues with RTOS threads To fix this we make this a "critical task" so that
  // it's not interrupted. Additionally, "Sampling time" should be set no lower
  // than 64.5 per channel if you haven't already
  taskENTER_CRITICAL();

  // Get the ADC resolution
  int resolution = getADCResolution(hadc1);

  // Start the ADC and return if not successful
  if (HAL_ADC_Start(hadc1) != HAL_OK) {
    return 1;
  }

  // Loop through the channels
  for (int i = 0; i < channels; i++) {
    // Wait for the ADC conversion to complete
    HAL_StatusTypeDef debugStatus;
    debugStatus = HAL_ADC_PollForConversion(hadc1, maxDelayPerChannel);
    if (debugStatus != HAL_OK) {
      return 1;
    }
    // Get the ADC value
    // This will also increment the ADC channel for the next time we call
    // HAL_ADC_PollForConversion()
    uint32_t adcValue = HAL_ADC_GetValue(hadc1);

    // Convert to voltage and then store in adcValues
    adcValues[i] = (((float)adcValue / resolution) * 3.3);
  }
  // Stop the ADC and return if not successful
  if (HAL_ADC_Stop(hadc1) != HAL_OK) {
    return 1;
  }
  // Exit critical task
  taskEXIT_CRITICAL();
  return 0;
}

// Gives the voltage and time-stamps of all ADCs (0-3.3V) in an array that
// contains all channels
int readADC(ADC_HandleTypeDef* hadc1, int channels, float* adcValues,
            int maxDelayPerChannel) {
  // This function relies on HAL switching to the next ADC rank, but this causes
  // issues with RTOS threads To fix this we make this a "critical task" so that
  // it's not interrupted. Additionally, "Sampling time" should be set no lower
  // than 64.5 per channel if you haven't already
  taskENTER_CRITICAL();

  // Get the ADC resolution
  int resolution = getADCResolution(hadc1);

  // Start the ADC and return if not successful
  if (HAL_ADC_Start(hadc1) != HAL_OK) {
    return 1;
  }

  // Loop through the channels
  for (int i = 0; i < channels; i++) {
    // Wait for the ADC conversion to complete
    HAL_StatusTypeDef debugStatus;
    debugStatus = HAL_ADC_PollForConversion(hadc1, maxDelayPerChannel);
    if (debugStatus != HAL_OK) {
      return 1;
    }
    // Get the ADC value
    // This will also increment the ADC channel for the next time we call
    // HAL_ADC_PollForConversion()
    uint32_t adcValue = HAL_ADC_GetValue(hadc1);

    // Convert to voltage and then store in adcValues
    adcValues[i] = (((float)adcValue / resolution) * 3.3);
  }
  // Get the timestamp of when the readings were taken and put it as the last
  // value in the array
  uint64_t timestamp = getTimestamp();
  // Needs to be a float because it's a float array, but the return value is
  // uint64_t
  adcValues[channels] = (float)timestamp;

  // Stop the ADC and return if not successful
  if (HAL_ADC_Stop(hadc1) != HAL_OK) {
    return 1;
  }
  // Exit critical task
  taskEXIT_CRITICAL();
  return 0;
}
