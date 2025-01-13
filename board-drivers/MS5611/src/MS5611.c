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
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();
	
	MS5611_chipSelect(BAR);
	status=HAL_SPI_Receive(BAR->hspi, (uint8_t *)rx_buffer, num_bytes, BAR->SPI_TIMEOUT);
	MS5611_chipRelease(BAR);
	
	taskEXIT_CRITICAL();

	return status;
}

//Write register from barometer
HAL_StatusTypeDef MS5611_write(MS5611* BAR, uint8_t* tx_buffer, uint8_t num_bytes) {
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	MS5611_chipSelect(BAR);
	status = HAL_SPI_Transmit(BAR->hspi, (uint8_t *)tx_buffer, 1, BAR->SPI_TIMEOUT);
	MS5611_chipRelease(BAR);

	taskEXIT_CRITICAL();
	return status;
}

int MS5611_send(MS5611* BAR, uint8_t reg) {
	if (MS5611_write(BAR, &reg, 1) == HAL_OK) {
		return 1;
	} else {
		return 0;
	}
}

uint8_t MS5611_recieve(MS5611* BAR, uint8_t reg, uint8_t rx_num_bytes) {
	assert(rx_num_bytes <= 4); // we shouldn't need to read more than 24bits tbh
	uint8_t rx_buffer[4]; // i don't wanna do malloc on this shi
	if (MS5611_read(BAR, reg, &rx_buffer, rx_num_bytes) == HAL_OK) {
		return rx_buffer;
	} else {
		return 0;
	}
}

// Software and memory reset
int MS5611_Reset(MS5611* BAR) {
	if (!MS5611_send(BAR, MS5611_RESET)) {
		osDelay(5); // Time for reset
    	return 0;
	}
	return 1;
}

// Read the programmable read only memory
int MS5611_readPROM(MS5611* BAR) {

}

//Pressure convert
float MS5611_presConvert() {

}

//Altitude convert
float MS5611_tempConvert() {

}

int MS5611_compensateTemp(float* temp, float* pres) {
	
}

//Get pressure from barometer
int MS5611_getPres(MS5611* BAR, float* pres) {

}

//Get angular rate from barometer
int MS5611_getAlt(MS5611* BAR, float* alt, float BAR_SEA_LEVEL_PRESS) {

}