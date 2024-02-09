/*
 * LPS22HBTR.c
 *
 *  Created on: February 9, 2024
 *      Author: jackmh
 */

#include "../inc/LPS22HBTR.h"

//Pressure convert
float BAR_presConvert(uint8_t H_Byte, uint8_t L_Byte) {
    int16_t pres_raw = (int16_t)(H_Byte << 8) + L_Byte; // Combine the two bytes into one 16 bit number. This is already formatted for an int16_t
}

//Altitude convert
float BAR_altConvert(uint8_t H_Byte, uint8_t L_Byte) {
    int16_t alt_raw = (int16_t)(H_Byte << 8) + L_Byte; // Combine the two bytes into one 16 bit number. This is already formatted for an int16_t
}

//Chip select
void BAR_chipSelect(BAR* BAR) {
	HAL_GPIO_WritePin(BAR->CS_GPIO_Port, BAR->CS_GPIO_Pin, 0); //CS pin
}

//Chip release
void BAR_chipRelease(BAR* BAR) {
	HAL_GPIO_WritePin(BAR->CS_GPIO_Port, BAR->CS_GPIO_Pin, 1); //CS pin
}

//Read register from barometer
HAL_StatusTypeDef BAR_read(BAR* BAR, uint8_t reg_addr, uint8_t* rx_buffer, uint8_t num_bytes) {
	uint8_t reg_buffer[1] = {reg_addr | 0x80};  // Set register for reading

	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	BAR_chipSelect(BAR);
	HAL_SPI_Transmit(BAR->hspi, (uint8_t *)reg_buffer, 1, BAR->SPI_TIMEOUT);
	status = HAL_SPI_Receive(BAR->hspi, (uint8_t *)rx_buffer, num_bytes, BAR->SPI_TIMEOUT);
	BAR_chipRelease(BAR);

	taskEXIT_CRITICAL();
	return status;
}

//Write register from barometer
HAL_StatusTypeDef BAR_write(BAR* BAR, uint8_t* tx_buffer, uint8_t num_bytes) {
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	BAR_chipSelect(BAR);
	status = HAL_SPI_Transmit(BAR->hspi, (uint8_t *)tx_buffer, num_bytes + 1, BAR->SPI_TIMEOUT);
	BAR_chipRelease(BAR);

	taskEXIT_CRITICAL();
	return status;
}

//Initialize barometer
int BAR_init(BAR* BAR) {
}

//Get pressure from barometer
int BAR_getPres(BAR* BAR, float* pres) {
}

//Get altitude from barometer
int BAR_getAlt(BAR* BAR, float* alt) {
}

//Send command to barometer
int BAR_send(BAR* BAR, uint8_t cmd, uint8_t value) {
}
