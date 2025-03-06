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
	#include "semphr.h"
#else
    #error "This library is designed for use with FreeRTOS. Please include the FreeRTOS library in your project."
#endif

#define TC_TASK_PRIORITY				1

// Constants
//Hard-Coded
#define ADS_PGA_ENABLED            		(uint8_t)0x00 // Default
#define ADS_MODE_NORMAL            		(uint8_t)0x00 // Default
#define ADS_CONV_MODE_CONT            	(uint8_t)0x04
#define ADS_CONV_MODE_SING            	(uint8_t)0x00 // Default
#define ADS_INTERNAL_TEMP_DISABLED      (uint8_t)0x00 // Default
#define ADS_INTERNAL_TEMP_ENABLED     	(uint8_t)0x02
#define ADS_BURN_OUT_DISABLED           (uint8_t)0x00 // Default
#define ADS_VOLT_REF_EXT_REF0           (uint8_t)0x40
#define ADS_VOLT_REF_INT           		(uint8_t)0x00 // Default
#define ADS_FILTER_DISABLED           	(uint8_t)0x00 // Default
#define ADS_PSW_DISABLED            	(uint8_t)0x00 // Default
#define ADS_IDAC_DISABLED            	(uint8_t)0x00 // Default
#define ADS_l1MUX_DISABLED            	(uint8_t)0x00 // Default
#define ADS_l2MUX_DISABLED            	(uint8_t)0x00 // Default
#define ADS_DRDY_MODE_ONLY_DRDY         (uint8_t)0x00
#define ADS_DRDY_MODE_BOTH				(uint8_t)0x02 // Default

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

#define ADS_INTERNAL_TEMP_RATE			ADS_DATA_RATE_175
#define ADS_INTERNAL_TEMP_DELAY			(uint32_t)500 // ms between internal temp readings
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
/*typedef struct {
    SPI_HandleTypeDef* hspi;
    uint16_t SPI_TIMEOUT;
    GPIO_TypeDef * CS_GPIO_Port;
    uint16_t CS_GPIO_Pin;
    uint8_t mux;

    uint8_t gain;
    uint8_t rate;
    double cold_junction_voltage; // microvolts
} ADS1120;*/

typedef struct {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef *DOUT_GPIO_Port;
	uint16_t DOUT_GPIO_Pin;
	uint16_t SPI_TIMEOUT;
	GPIO_TypeDef *CS_GPIO_Port;
	uint16_t CS_GPIO_Pin;
	uint8_t mux;

	uint8_t gain;
	uint8_t rate;
	double step_size;

	int16_t current_raw;
	uint64_t timestamp;
	SemaphoreHandle_t reading_semaphore;
} ADS_TC_t;

typedef struct {
	TaskHandle_t tc_task;
	ADS_TC_t *TCs;
	uint8_t TC_count;


	int16_t raw_temp;
	SemaphoreHandle_t temp_semaphore;
	uint32_t last_temp;
} ADS_Main_t;

typedef struct {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef *DOUT_GPIO_Port;
	uint16_t DOUT_GPIO_Pin;
	uint16_t SPI_TIMEOUT;
	GPIO_TypeDef *CS_GPIO_Port;
	uint16_t CS_GPIO_Pin;

	ADS_TC_t* muxes[2];
	uint8_t last_state;
	uint8_t mux_count;
	uint8_t current_mux;
	uint32_t last_tick;
	uint32_t timer_start;
} ADS_Chip;

typedef struct {
	float temp_c;
	uint64_t timestamp;
	uint8_t error;
} ADS_Reading_t;

double ADS_convertRawToMicrovolts(ADS_TC_t *ADS, int16_t raw); //DONE

// Microvolts in -> Celsius out
float ADS_polyMicrovoltsToTemp(double microvolts); //DONE

// Celsius in -> Microvolts out
double ADS_polyTempToMicrovolts(float temp); //DONE

// Configures each TC struct, does not actually communicate with the chip
void ADS_configTC(ADS_TC_t *ADSTC, SPI_HandleTypeDef* hspi, GPIO_TypeDef *DOUT_GPIO_Port, uint16_t DOUT_GPIO_Pin, uint16_t SPI_TIMEOUT, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint8_t mux, uint8_t gain, uint8_t rate); //DONE

void ADSTC_chipSelect(ADS_TC_t *ADS); //DONE

void ADSTC_chipRelease(ADS_TC_t *ADS); //DONE


int ADS_init(ADS_Main_t *ADSMain, ADS_TC_t *TCs, uint8_t num_TCs); //DONE

void vTCTask(void *pvParameters);
// Configure byte 3 and 4 of chip, save cjc
// reference_volts is in volts, cold_junction_temp is in celsius
//int ADS_init(ADS1120 *ADS, uint8_t mux, uint8_t gain, uint8_t rate, float cold_junction_temp);

HAL_StatusTypeDef ADS_write(ADS_Chip *ADS, uint8_t *tx_buffer, uint8_t num_bytes); //DONE

HAL_StatusTypeDef ADS_read(ADS_Chip *ADS, uint8_t reg_addr, uint8_t *rx_buffer, uint8_t num_bytes); //DONE

void ADS_chipSelect(ADS_Chip *ADS); //DONE

void ADS_chipRelease(ADS_Chip *ADS); //DONE

int ADS_configure(ADS_Chip *ADS, uint8_t TC_index, int check); //DONE

int ADS_configureChip(ADS_Chip *ADS, int check);

int ADS_switchConf(ADS_Chip *ADS, uint8_t TC_index);

//int ADS_readTemp(ADS1120 *ADS, float *temp);

// read from one TC, no timestamp TC_num IS 0 INDEXED
int ADS_readIndividual(ADS_Main_t *ADSMain, uint8_t TC_index, float *reading); //DONE

// TC_num IS 0 INDEXED
int ADS_readIndividualwTimestamp(ADS_Main_t *ADSMain, uint8_t TC_index, ADS_Reading_t *reading); // TODO ALMOST DONE

// The array at readings must be at least as long as the number of TCs that were originally passed to ADS_init
// Readings are returned in the order that they were passed to ADS_init
// returns 1 if unable to take internal temp semaphore, all readings are invalid. If a reading is NAN, then its semaphore couldn't be taken.
int ADS_readAll(ADS_Main_t *ADSMain, float *readings); // TODO ALMOST DONE

// The array at readings must be at least as long as the number of TCs that were originally passed to ADS_init
int ADS_readAllwTimestamps(ADS_Main_t *ADSMain, ADS_Reading_t *readings);

int ADS_readInternalTemp(ADS_Main_t *ADSMain, float *temp);

#endif
