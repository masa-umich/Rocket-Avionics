/**
 * ADS1120.h
 *
 *  Created on: November 24, 2024
 *      Author: Felix Foreman-Braunschweig
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

// Configuration constants
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

// Mux: the combination of pins on the ADS1120 that a thermocouple could be connected to. AIN0, AIN1, AIN2, and AIN3 all correspond to pins on the ADS1120
#define ADS_MUX_AIN0_AIN1            	(uint8_t)0x00 // AIN0 is positive, AIN1 is negative
#define ADS_MUX_AIN0_AIN2            	(uint8_t)0x10 // AIN0 is positive, AIN2 is negative
#define ADS_MUX_AIN0_AIN3            	(uint8_t)0x20 // AIN0 is positive, AIN3 is negative
#define ADS_MUX_AIN1_AIN2            	(uint8_t)0x30 // AIN1 is positive, AIN2 is negative
#define ADS_MUX_AIN1_AIN3            	(uint8_t)0x40 // AIN1 is positive, AIN3 is negative
#define ADS_MUX_AIN2_AIN3            	(uint8_t)0x50 // AIN2 is positive, AIN3 is negative
#define ADS_MUX_AIN1_AIN0            	(uint8_t)0x60 // AIN1 is positive, AIN0 is negative
#define ADS_MUX_AIN3_AIN2            	(uint8_t)0x70 // AIN3 is positive, AIN2 is negative

// Gain
#define ADS_PGA_GAIN_1            		(uint8_t)0x00 // 1x Gain
#define ADS_PGA_GAIN_2            		(uint8_t)0x02 // 2x Gain
#define ADS_PGA_GAIN_4            		(uint8_t)0x04 // 4x Gain
#define ADS_PGA_GAIN_8            		(uint8_t)0x06 // 8x Gain
#define ADS_PGA_GAIN_16            		(uint8_t)0x08 // 16x Gain
#define ADS_PGA_GAIN_32            		(uint8_t)0x0A // 32x Gain
#define ADS_PGA_GAIN_64            		(uint8_t)0x0C // 64x Gain
#define ADS_PGA_GAIN_128            	(uint8_t)0x0E // 128x Gain

// Datarate / polling rate
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


/**
 * ADS_TC_t stores information about a specific thermocouple connection.
 * This includes information about the specific ADS1120 chip 
 * that the thermocouple is connected to (SPI bus, CS pin), which ADS1120 "channel"
 * it's connected to, and configuration information like the gain and datarate.
 * It also stores the latest reading and associated timestamp from the thermocouple.
 */
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

/**
 * ADS_Main_t is the main handle for everything.
 * It stores the RTOS task handle for the task that collects
 * thermocouple measurements, pointers to all of the 
 * individual thermocouple connections, and the current cold junction temperature.
 */
typedef struct {
	TaskHandle_t tc_task;
	ADS_TC_t *TCs;
	uint8_t TC_count;


	int16_t raw_temp;
	SemaphoreHandle_t temp_semaphore;
	uint32_t last_temp;
} ADS_Main_t;

/**
 * ADS_Chip contains information for each ADS1120 chip.
 * It stores SPI bus information, pointers to the thermocouples
 * that are connected to the chip, and timing information
 * related to reading from the chip and switching between thermocouple connections.
 */
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

/**
 * ADS_Reading_t packages a single thermocouple reading along with
 * the timestamp when it was read, and an error flag.
 * 
 * temp_c is the cold junction compensated temperature in Celsius
 * timestamp is the time when the reading was received from the ADS1120.
 * It is in whatever format was given by the external function getTimestamp().
 * 
 * If error is 0, then temp_c and timestamp are valid, otherwise
 * there was an issue reading the latest measurement - temp_c and timestamp
 * will not be valid and could be anything.
 */
typedef struct {
	float temp_c;
	uint64_t timestamp;
	uint8_t error;
} ADS_Reading_t;


/**
 * Convert a raw 16 bit value from a ADS1120 to a microvolt reading.
 *
 * @param ADS	The thermocouple that this reading came from
 * @param raw	The 16 bit value from the ADS1120
 * @return		The voltage measured by the ADS1120 in microvolts
 */
double ADS_convertRawToMicrovolts(ADS_TC_t *ADS, int16_t raw);

/**
 * Convert a voltage of a type T thermocouple to its corresponding temperature.
 * 
 * @param microvolts	The voltage of the thermocouple in microvolts
 * @return				The calculated temperature of the thermocouple in Celsius
 */
float ADS_polyMicrovoltsToTemp(double microvolts);

/**
 * Convert a temperature of a type T thermocouple to the voltage it would read.
 * 
 * @param temp	The temperature of the thermocouple in Celsius
 * @return		The calculated voltage in microvolts that the thermocouple should generate
 * 				if it was at the given temperature
 */
double ADS_polyTempToMicrovolts(float temp);

/**
 * Configure a thermocouple with connection information, gain, and datarate.
 * 
 * The DOUT (MISO) pin of the SPI bus is needed since it's used as a GPIO pin 
 * between SPI communications to determine if data is ready.
 * NOTE: this does not actually communicate with the chip, it just sets up the struct.
 * 
 * @param ADSTC				The thermocouple to configure
 * @param hspi				The SPI handle that the corresponding ADS1120 is connected to
 * @param DOUT_GPIO_Port	The GPIO port that the MISO pin of the SPI bus is connected to
 * @param DOUT_GPIO_Pin		The GPIO pin that the MISO pin of the SPI bus is connected to
 * @param SPI_TIMEOUT		The timeout to use for SPI communications (milliseconds)
 * @param CS_GPIO_Port		The GPIO port that the chip select pin of the ADS1120 is connected to
 * @param CS_GPIO_Pin		The GPIO pin that the chip select pin of the ADS1120 is connected to
 * @param mux				The combination of pins on the ADS1120 that the thermocouple is connected to.
 * @param gain				The gain to use for this thermocouple
 * @param rate				The datarate to use for this thermocouple
 */
void ADS_configTC(ADS_TC_t *ADSTC, SPI_HandleTypeDef* hspi, GPIO_TypeDef *DOUT_GPIO_Port, uint16_t DOUT_GPIO_Pin, uint16_t SPI_TIMEOUT, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint8_t mux, uint8_t gain, uint8_t rate);

/**
 * Configure the main struct and initialize the RTOS task.
 * 
 * @param ADSMain	The main handle where the RTOS task handle and all thermocouple pointers should be stored
 * @param TCs		All of the thermocouples to be monitored by the RTOS task. This should be a pointer to an array of thermocouples
 * @param num_TCs	The number of thermocouples to be monitored by the RTOS task
 * @return			The status of the created RTOS task. Returns 0 if the task started successfully, returns 1 if the task failed to start
 */
int ADS_init(ADS_Main_t *ADSMain, ADS_TC_t *TCs, uint8_t num_TCs);

/**
 * The main function for the RTOS task.
 * Collects and stores readings from all thermocouples, including a cold junction temperature 
 * from one of the connected ADS1120s
 * 
 * @param pvParameters		The pointer to the main ADS_Main_t handle
 */
void vTCTask(void *pvParameters);

/**
 * Configure an ADS1120 to read from one of its connected thermocouples.
 * This can also be used to configure an ADS1120 to read from its internal temperature sensor
 *  
 * @param ADS		The specific ADS1120 chip to configure
 * @param TC_index	The index of the thermocouple to set up the ADS1120 to read from.
 * 					This is specific to how the ADS chip struct was set up, and since there can
 * 					be up to 2 thermocouples connected to each chip, this parameter can only be 0 or 1.
 * 					If this parameter is 2, the ADS1120 is configured to read from its internal temperature sensor
 * @param check		If this is 1, the configuration of the ADS1120 will be checked after it's written to make sure 
 * 					the new configuration was saved. If this is 0, no verification is done.
 * @return			The result of the verification. Returns 0 if the verification passes and the configuration on 
 * 					the ADS1120 matches the passed configuration and returns 1 if the configuration fails and the ADS1120
 * 					configuration remains the same as before this function was called. If check is 0, then this always returns 0.
 */
int ADS_configure(ADS_Chip *ADS, uint8_t TC_index, int check);

/**
 * Configure an ADS1120 chip with the default configuration
 * 
 * @param ADS		The specific ADS1120 chip to configure
 * @param check		If this is 1, the configuration of the ADS1120 will be checked after it's written to make sure 
 * 					the new configuration was saved. If this is 0, no verification is done.
 * @return			The result of the verification. Returns 0 if the verification passes and the configuration on 
 * 					the ADS1120 matches the passed configuration and returns 1 if the configuration fails and the ADS1120
 * 					configuration remains the same as before this function was called. If check is 0, then this always returns 0.
 */
int ADS_configureChip(ADS_Chip *ADS, int check);

/**
 * Get the latest temperature of one thermocouple
 * This reading is cold junction compensated using one of the connected ADS1120s
 * 
 * @param ADSMain	The main driver handle
 * @param TC_index	The index of the thermocouple to be read. This is 0 indexed and is based on the order
 * 					of the array that was originally passed to ADS_init
 * @param reading	The pointer to where the reading should be stored. This is a temperature measurement in Celsius.
 * @return			Returns 0 if retrieving the reading was successful, returns 1 if the reading could not be retrieved.
 */
int ADS_readIndividual(ADS_Main_t *ADSMain, uint8_t TC_index, float *reading);

/**
 * Get the latest temperature of one thermocouple, including the timestamp when it was collected
 * This reading is cold junction compensated using one of the connected ADS1120s
 * 
 * The error flag in the passed ADS_Reading_t reflects the success of this function, and either it or the return value can be used to determine whether or not
 * the function successfully retrieved the reading.
 * 
 * The timestamp of the reading is taken when the reading is received from the ADS1120 NOT when this function is called. This means it can be 
 * used to determine if everything is working and readings are being updated. For example, if the datarate is set at 20hz, and the timestamp is 
 * significantly greater than 50ms in the past, the reading is probably not getting updated.
 * 
 * @param ADSMain	The main driver handle
 * @param TC_index	The index of the thermocouple to be read. This is 0 indexed and is based on the order
 * 					of the array that was originally passed to ADS_init
 * @param reading	The pointer to where the reading and timestamp should be stored
 * @return			Returns 0 if retrieving the reading was successful, returns 1 if the reading could not be retrieved.
 */
int ADS_readIndividualwTimestamp(ADS_Main_t *ADSMain, uint8_t TC_index, ADS_Reading_t *reading);

/**
 * Get the latest temperature of all thermocouples
 * These readings are cold junction compensated using one of the connected ADS1120s
 * 
 * Readings are in the order that their corresponding thermocouple was passed to ADS_init
 * 
 * If a reading couldn't be retrieved, its reading will be NAN.
 * Even if this function returns 0, some or all of the readings could not have been retrieved. The individual readings will reflect
 * if they weren't successful (NAN). If this function returns 1, then all readings are not valid.
 * 
 * @param ADSMain	The main driver handle
 * @param readings	The pointer to the array where the readings should be stored. This should be at least as long as the number of 
 * 					thermocouples that were passed to ADS_init
 * @return			Returns 0 if retrieving the internal temperature was successful, returns 1 if the internal temperature could not be 
 * 					retrieved.
 */
int ADS_readAll(ADS_Main_t *ADSMain, float *readings);

/**
 * Get the latest temperature of all thermocouples
 * These readings are cold junction compensated using one of the connected ADS1120s
 * 
 * Readings are in the order that their corresponding thermocouple was passed to ADS_init
 * 
 * If a reading couldn't be retrieved, its error flag will be 1.
 * Even if this function returns 0, some or all of the readings could not have been retrieved. The individual readings will reflect
 * if they weren't successful (error flag = 1). If this function returns 1, then all readings are not valid, although their error flags might not reflect this.
 * 
 * Just as in ADS_readIndividualwTimestamp(), the timestamps are taken when the readings are received from the ADS1120s, so they can be used to determine if
 * readings are being updated.
 * 
 * @param ADSMain	The main driver handle
 * @param readings	The pointer to the array where the readings and timestamps should be stored. This should be at least as long as the number of 
 * 					thermocouples that were passed to ADS_init
 * @return			Returns 0 if retrieving the internal temperature was successful, returns 1 if the internal temperature could not be 
 * 					retrieved.
 */
int ADS_readAllwTimestamps(ADS_Main_t *ADSMain, ADS_Reading_t *readings);

/**
 * Get the latest internal temperature from one of the ADS1120s.
 * By default this is updated twice a second, but it can be changed by changing the ADS_INTERNAL_TEMP_DELAY macro.
 * This is the temperature used for cold junction compensation in all read functions. The specific ADS1120 that this is 
 * retrieved from is whichever ADS1120 is connected to the first thermocouple passed to ADS_init.
 * 
 * @param ADSMain	The main driver handle
 * @param temp		The pointer to where the temperature should be stored. This temperature is in Celsius.
 * @return			Returns 0 if retrieving the temperature was successful, returns 1 if it could not be 
 * 					retrieved.
 */
int ADS_readInternalTemp(ADS_Main_t *ADSMain, float *temp);



void ADSTC_chipSelect(ADS_TC_t *ADS);

void ADSTC_chipRelease(ADS_TC_t *ADS);

// Configure byte 3 and 4 of chip, save cjc
// reference_volts is in volts, cold_junction_temp is in celsius
//int ADS_init(ADS1120 *ADS, uint8_t mux, uint8_t gain, uint8_t rate, float cold_junction_temp);

HAL_StatusTypeDef ADS_write(ADS_Chip *ADS, uint8_t *tx_buffer, uint8_t num_bytes);

HAL_StatusTypeDef ADS_read(ADS_Chip *ADS, uint8_t reg_addr, uint8_t *rx_buffer, uint8_t num_bytes);

void ADS_chipSelect(ADS_Chip *ADS);

void ADS_chipRelease(ADS_Chip *ADS);

int ADS_switchConf(ADS_Chip *ADS, uint8_t TC_index);

#endif

/* NOTES FOR README
If a read function fails to retrieve a reading, it doesn't mean that a new reading isn't ready. At the same time, you can call the read functions as often as you want, 
if a new reading isn't ready it just returns the previous reading. Basically, whether or not a read function fails has nothing to do with if data is ready on the ADS1120 side.
*/