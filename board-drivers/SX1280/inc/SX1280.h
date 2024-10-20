/*
 * SX1280.h
 *
 *  Created on: September 29, 2024
 *      Author: felixfb
 */

#ifndef SX1280_H	// Begin header include protection
#define SX1280_H

#include "main.h"
#include <math.h>

#ifndef FreeRTOS_H
    #include "FreeRTOS.h"
    #include "task.h"
#else
    #error "This library is designed for use with FreeRTOS. Please include the FreeRTOS library in your project."
#endif

// Begin register map
#define SX_STATUS_OPCODE		    	(uint8_t)0xC0
#define SX_WRITE_BUFFER_OPCODE		    (uint8_t)0x1A
#define SX_READ_BUFFER_OPCODE		    (uint8_t)0x1B
#define SX_SET_STANDBY_OPCODE		    (uint8_t)0x80
#define SX_SET_TX_OPCODE		    	(uint8_t)0x83
#define SX_SET_RX_OPCODE		    	(uint8_t)0x82
#define SX_SET_RX_CONT_OPCODE		    (uint8_t)0x94
#define SX_SET_LONG_PREAMBLE_OPCODE		(uint8_t)0x9B
#define SX_SET_PACKET_TYPE_OPCODE		(uint8_t)0x8A
#define SX_SET_RF_FREQ_OPCODE		    (uint8_t)0x86
#define SX_SET_TX_CONF_OPCODE		    (uint8_t)0x8E
#define SX_SET_BUFFER_ADDRESS_OPCODE	(uint8_t)0x8F
#define SX_SET_MODULATION_CONF_OPCODE	(uint8_t)0x8B
#define SX_SET_PACKET_CONF_OPCODE		(uint8_t)0x8C
#define SX_GET_RX_BUFFER_STATUS_OPCODE	(uint8_t)0x17
#define SX_GET_PACKET_STATUS_OPCODE		(uint8_t)0x1D
#define SX_GET_RSSI_OPCODE		    	(uint8_t)0x1F
// End register map

// Constants
#define SX_STANDBY_CONF_RC            	(uint8_t)0x00
#define SX_STANDBY_CONF_XOSC           	(uint8_t)0x01
#define SX_TIME_STEP_15_625_US      	(uint8_t)0x00 // 15.625 uS
#define SX_TIME_STEP_6_25_US        	(uint8_t)0x01 // 6.25 uS
#define SX_TIME_STEP_1_MS     			(uint8_t)0x02 // 1 mS
#define SX_TIME_STEP_4_MS      			(uint8_t)0x03 // 4 mS
#define SX_PACKET_TYPE_LORA     		(uint8_t)0x01
#define SX_RAMP_02_US     				(uint8_t)0x00
#define SX_RAMP_04_US     				(uint8_t)0x20
#define SX_RAMP_06_US     				(uint8_t)0x40
#define SX_RAMP_08_US     				(uint8_t)0x60
#define SX_RAMP_10_US     				(uint8_t)0x80
#define SX_RAMP_12_US     				(uint8_t)0xA0
#define SX_RAMP_16_US     				(uint8_t)0xC0
#define SX_RAMP_20_US     				(uint8_t)0xE0





typedef struct {
    //SPI stuff
    SPI_HandleTypeDef* hspi;
    uint16_t SPI_TIMEOUT;
    GPIO_TypeDef * CS_GPIO_Port;
    uint16_t CS_GPIO_Pin;

    //Offset values
    float pres_offset;
    float alt_offset;
} BAR;

//Pressure convert
float BAR_presConvert(uint8_t XL_Byte, uint8_t L_Byte, uint8_t H_Byte);

//Altitude convert
float BAR_altConvert(uint8_t XL_Byte, uint8_t L_Byte, uint8_t H_Byte, float BAR_SEA_LEVEL_PRESS);

//Temperature convert
float BAR_tempConvert(uint8_t L_Byte, uint8_t H_Byte);

//Chip select
void BAR_chipSelect(BAR* BAR);

//Chip release
void BAR_chipRelease(BAR* BAR);

//Read register from barometer
HAL_StatusTypeDef BAR_read(BAR* BAR, uint8_t reg_addr, uint8_t* rx_buffer, uint8_t num_bytes);

//Write register from barometer
HAL_StatusTypeDef BAR_write(BAR* BAR, uint8_t* tx_buffer, uint8_t num_bytes);

//Initialize barometer
int BAR_init(BAR* BAR);

//Get pressure from barometer
int BAR_getPres(BAR* BAR, float* pres);

//Get angular rate from barometer
int BAR_getAlt(BAR* BAR, float* alt, float BAR_SEA_LEVEL_PRESS);

//Get temperature from barometer
int BAR_getTemp(BAR* BAR, float* temp);

//Send command to barometer
int BAR_send(BAR* BAR, uint8_t cmd, uint8_t value);

//Return 0 is WHO_AM_I register can be read, 1 otherwise
int BAR_whoami(BAR* BAR);

//Waits / blocks for the pressure data to be ready
int BAR_waitForPres(BAR* BAR);

//Waits / blocks for the temperature data to be ready
int BAR_waitForTemp(BAR* BAR);

// Software and memory reset
int BAR_Reset(BAR* BAR);

#endif
