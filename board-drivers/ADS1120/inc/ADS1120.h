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
#define ADS_CONV_MODE_CONT            	(uint8_t)0x04
#define ADS_CONV_MODE_SING            	(uint8_t)0x00 // Default
#define ADS_INTERNAL_TEMP_DISABLED      (uint8_t)0x00 // Default
#define ADS_BURN_OUT_DISABLED           (uint8_t)0x00 // Default
#define ADS_VOLT_REF_EXT_REF0           (uint8_t)0x40
#define ADS_VOLT_REF_INT           		(uint8_t)0x00 // Default
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

// Type T thermocouple constants
// Temperature -> Voltage when temperature > 0
#define ADS_POLY_POS_C1					(double)3.8748106364e1
#define ADS_POLY_POS_C2					(double)3.329222788e-2
#define ADS_POLY_POS_C3					(double)2.0618243404e-4
#define ADS_POLY_POS_C4					(double)-2.1882256846e-6
#define ADS_POLY_POS_C5					(double)1.0996880928e-8
#define ADS_POLY_POS_C6					(double)-3.0815758772e-11
#define ADS_POLY_POS_C7					(double)4.547913529e-14
#define ADS_POLY_POS_C8					(double)-2.7512901673e-17

// Temperature -> Voltage when temperature < 0
#define ADS_POLY_NEG_C1					(double)3.8748106364e1
#define ADS_POLY_NEG_C2					(double)4.4194434347e-2
#define ADS_POLY_NEG_C3					(double)1.1844323105e-4
#define ADS_POLY_NEG_C4					(double)2.0032973554e-5
#define ADS_POLY_NEG_C5					(double)9.0138019559e-7
#define ADS_POLY_NEG_C6					(double)2.2651156593e-8
#define ADS_POLY_NEG_C7					(double)3.6071154205e-10
#define ADS_POLY_NEG_C8					(double)3.8493939883e-12
#define ADS_POLY_NEG_C9					(double)2.8213521925e-14
#define ADS_POLY_NEG_C10				(double)1.4251594779e-16
#define ADS_POLY_NEG_C11				(double)4.8768662286e-19
#define ADS_POLY_NEG_C12				(double)1.079553927e-21
#define ADS_POLY_NEG_C13				(double)1.3945027062e-24
#define ADS_POLY_NEG_C14				(double)7.9795153927e-28

// Voltage -> Temperature when voltage > 0
#define ADS_POLY_INV_POS_C1				(double)2.5928e-2
#define ADS_POLY_INV_POS_C2				(double)-7.602961e-7
#define ADS_POLY_INV_POS_C3				(double)4.637791e-11
#define ADS_POLY_INV_POS_C4				(double)-2.165394e-15
#define ADS_POLY_INV_POS_C5				(double)6.048144e-20
#define ADS_POLY_INV_POS_C6				(double)-7.293422e-25

// Voltage -> Temperature when voltage < 0
#define ADS_POLY_INV_NEG_C1				(double)2.5929192e-2
#define ADS_POLY_INV_NEG_C2				(double)-2.1316967e-7
#define ADS_POLY_INV_NEG_C3				(double)7.9018692e-10
#define ADS_POLY_INV_NEG_C4				(double)4.2527777e-13
#define ADS_POLY_INV_NEG_C5				(double)1.3304473e-16
#define ADS_POLY_INV_NEG_C6				(double)2.0241446e-20
#define ADS_POLY_INV_NEG_C7				(double)1.2668171e-24


// This represents a connection to a thermocouple NOT a chip.
// Two different instances of this could represent connections on the same chip, but different input pins (mux)
typedef struct {
    SPI_HandleTypeDef* hspi;
    uint16_t SPI_TIMEOUT;
    GPIO_TypeDef * CS_GPIO_Port;
    uint16_t CS_GPIO_Pin;
    uint8_t mux;

    uint8_t gain;
    uint8_t rate;
    double cold_junction_voltage; // microvolts
} ADS1120;


double ADS_convertRawToMicrovolts(ADS1120 *ADS, uint8_t MSB, uint8_t LSB); //DONE

// Microvolts in -> Celsius out
float ADS_polyMicrovoltsToTemp(double microvolts); //DONE

// Celsius in -> Microvolts out
double ADS_polyTempToMicrovolts(float temp); //DONE

// Configure byte 3 and 4 of chip, save cjc
// reference_volts is in volts, cold_junction_temp is in celsius
int ADS_init(ADS1120 *ADS, uint8_t mux, uint8_t gain, uint8_t rate, float cold_junction_temp);

HAL_StatusTypeDef ADS_write(ADS1120 *ADS, uint8_t *tx_buffer, uint8_t num_bytes); //DONE

HAL_StatusTypeDef ADS_read(ADS1120 *ADS, uint8_t reg_addr, uint8_t *rx_buffer, uint8_t num_bytes); //DONE

void ADS_chipSelect(ADS1120 *ADS); //DONE

void ADS_chipRelease(ADS1120 *ADS); //DONE

int ADS_configure(ADS1120 *ADS, uint8_t mux, uint8_t gain, uint8_t rate, int check); //DONE

int ADS_configureChip(ADS1120 *ADS, int check);

int ADS_switchConf(ADS1120 *ADS);

// Do all of the following while holding CS low: switch to correct conf, either monitor DOUT and then clock in reading and send an extra 16 SCLKs, or delay time and then use RDATA to get reading, then let go of CS
// convert to microvolts, offset cjc voltage, convert to temp
int ADS_readTemp(ADS1120 *ADS, float *temp);

//Either call ADS_readTemp multiple times, this is the easiest, but if more speed is needed, use the method of switching conf as a reading is being clocked in
// ADS_readMultiple
#endif
