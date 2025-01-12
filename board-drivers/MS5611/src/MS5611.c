/*
 * MS5611.c
 *
 *  Created on: January 12, 2025
 *      Author: jackmh
 */

#include "MS5611.h"

//Chip select
void MS5611_chipSelect(MS5611* BAR) {
	HAL_GPIO_WritePin(BAR->CS_GPIO_Port, BAR->CS_GPIO_Pin, RESET); //CS pin
}

//Chip release
void MS5611_chipRelease(MS5611* BAR) {
	HAL_GPIO_WritePin(BAR->CS_GPIO_Port, BAR->CS_GPIO_Pin, SET); //CS pin
}

//Read register from barometer
HAL_StatusTypeDef MS5611_read(MS5611* BAR, uint8_t reg_addr, uint8_t* rx_buffer, uint8_t num_bytes) {

}

//Write register from barometer
HAL_StatusTypeDef MS5611_write(MS5611* BAR, uint8_t* tx_buffer, uint8_t num_bytes) {

}

// Software and memory reset
int MS5611_Reset(MS5611* BAR) {

}

// Read the programmable read only memory
int MS5611_readPROM(MS5611* BAR) {

}

//Pressure convert
float MS5611_presConvert(uint8_t XL_Byte, uint8_t L_Byte, uint8_t H_Byte) {

}

//Altitude convert
float MS5611_altConvert(uint8_t XL_Byte, uint8_t L_Byte, uint8_t H_Byte, float BAR_SEA_LEVEL_PRESS) {

}

//Temperature convert
float MS5611_tempConvert(uint8_t L_Byte, uint8_t H_Byte) {

}

//Initialize barometer
int MS5611_init(MS5611* BAR) {

}

//Get pressure from barometer
int MS5611_getPres(MS5611* BAR, float* pres) {

}

//Get angular rate from barometer
int MS5611_getAlt(MS5611* BAR, float* alt, float BAR_SEA_LEVEL_PRESS) {

}

//Get temperature from barometer
int MS5611_getTemp(MS5611* BAR, float* temp) {

}

//Send command to barometer
int MS5611_send(MS5611* BAR, uint8_t cmd, uint8_t value) {

}

//Waits / blocks for the pressure data to be ready
int MS5611_waitForPres(MS5611* BAR) {

}

//Waits / blocks for the temperature data to be ready
int MS5611_waitForTemp(MS5611* BAR) {

}