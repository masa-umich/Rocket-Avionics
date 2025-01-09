/*
 * ADS1120.h
 *
 *  Created on: November 24, 2024
 *      Author: felixfb
 */

#ifndef ADS1120_H
#define ADS1120_H

#include "main.h"
#include <math.h>

#ifndef FreeRTOS_H
    #include "FreeRTOS.h"
    #include "task.h"
#else
    #error "This library is designed for use with FreeRTOS. Please include the FreeRTOS library in your project."
#endif


// Constants
//Hard-Coded
#define ADS_PGA_ENABLED            		(uint8_t)0x00 // Default
#define ADS_MODE_NORMAL            		(uint8_t)0x00 // Default
#define ADS_CONV_MODE_CONT            	(uint8_t)0x04 // Default
#define ADS_INTERNAL_TEMP_DISABLED      (uint8_t)0x00 // Default
#define ADS_BURN_OUT_DISABLED           (uint8_t)0x00 // Default
#define ADS_VOLT_REF_EXT_REF0           (uint8_t)0x40 // Default
#define ADS_FILTER_DISABLED           	(uint8_t)0x00 // Default
#define ADS_PSW_DISABLED            	(uint8_t)0x00 // Default
#define ADS_IDAC_DISABLED            	(uint8_t)0x00 // Default
#define ADS_l1MUX_DISABLED            	(uint8_t)0x00 // Default
#define ADS_l2MUX_DISABLED            	(uint8_t)0x00 // Default
#define ADS_DRDY_MODE_ONLY_DRDY         (uint8_t)0x00 // Default

//Open
#define ADS_MUX_AIN0_AIN1            	(uint8_t)0x00 // AIN0 is positive, AIN1 is negative
#define ADS_MUX_AIN0_AIN2            	(uint8_t)0x10 // AIN0 is positive, AIN2 is negative
#define ADS_MUX_AIN0_AIN3            	(uint8_t)0x20 // AIN0 is positive, AIN3 is negative
#define ADS_MUX_AIN1_AIN2            	(uint8_t)0x30 // AIN1 is positive, AIN2 is negative
#define ADS_MUX_AIN1_AIN3            	(uint8_t)0x40 // AIN1 is positive, AIN3 is negative
#define ADS_MUX_AIN2_AIN3            	(uint8_t)0x50 // AIN2 is positive, AIN3 is negative
#define ADS_MUX_AIN1_AIN0            	(uint8_t)0x60 // AIN1 is positive, AIN0 is negative
#define ADS_MUX_AIN3_AIN2            	(uint8_t)0x70 // AIN3 is positive, AIN2 is negative
#define ADS_PGA_GAIN_1            		(uint8_t)0x00 // 1x Gain
#define ADS_PGA_GAIN_2            		(uint8_t)0x02 // 2x Gain
#define ADS_PGA_GAIN_4            		(uint8_t)0x04 // 4x Gain
#define ADS_PGA_GAIN_8            		(uint8_t)0x06 // 8x Gain
#define ADS_PGA_GAIN_16            		(uint8_t)0x08 // 16x Gain
#define ADS_PGA_GAIN_32            		(uint8_t)0x0A // 32x Gain
#define ADS_PGA_GAIN_64            		(uint8_t)0x0C // 64x Gain
#define ADS_PGA_GAIN_128            	(uint8_t)0x0E // 128x Gain
#define ADS_DATA_RATE_20            	(uint8_t)0x00 // 20 SPS ~ 20hz in normal mode
#define ADS_DATA_RATE_45            	(uint8_t)0x20 // 45 SPS ~ 45hz in normal mode
#define ADS_DATA_RATE_90            	(uint8_t)0x40 // 90 SPS ~ 90hz in normal mode
#define ADS_DATA_RATE_175            	(uint8_t)0x60 // 175 SPS ~ 175hz in normal mode
#define ADS_DATA_RATE_330            	(uint8_t)0x80 // 330 SPS ~ 330hz in normal mode
#define ADS_DATA_RATE_600            	(uint8_t)0xA0 // 600 SPS ~ 600hz in normal mode
#define ADS_DATA_RATE_1000            	(uint8_t)0xC0 // 1000 SPS ~ 1000hz in normal mode


typedef struct {
    SPI_HandleTypeDef* hspi;
    uint16_t SPI_TIMEOUT;
    GPIO_TypeDef * CS_GPIO_Port;
    uint16_t CS_GPIO_Pin;

    uint8_t gain;
    double cold_junction_voltage; // microvolts
    double voltage_ref; // microvolts
} ADS1120;


double ADS_convertRawToMicrovolts(ADS1120 *ADS, uint8_t MSB, uint8_t LSB); //DONE

float ADS_polyMicrovoltsToTemp(double microvolts);

double ADS_polyTempToMicrovolts(float temp);

// Reset chip, wait small delay maybe?, configure chip, save cjc and ref voltage, start conversion
// THIS FUNCTION IS BLOCKING, IT DELAYS FOR AT LEAST 1 MILLISECOND TO ENSURE THE CHIP HAS RESET
// reference_volts is in volts, cold_junction_temp is in celsius
int ADS_init(ADS1120 *ADS, uint8_t mux, uint8_t gain, uint8_t rate, double reference_volts, float cold_junction_temp);

HAL_StatusTypeDef ADS_write(ADS1120 *ADS, uint8_t *tx_buffer, uint8_t num_bytes); //DONE

HAL_StatusTypeDef ADS_read(ADS1120 *ADS, uint8_t reg_addr, uint8_t *rx_buffer, uint8_t num_bytes); //DONE

void ADS_chipSelect(ADS1120 *ADS); //DONE

void ADS_chipRelease(ADS1120 *ADS); //DONE

int ADS_configure(ADS1120 *ADS, uint8_t mux, uint8_t gain, uint8_t rate, int check); //DONE

// Read RDATA, convert to microvolts, offset cjc voltage, convert to temp
int ADS_readTemp(ADS1120 *ADS, float *temp);

#endif
