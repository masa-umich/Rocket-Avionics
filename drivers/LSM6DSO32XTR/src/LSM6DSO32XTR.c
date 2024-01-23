/*
 * LSM6DSO32XTR.c
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#include "../inc/LSM6DSO32XTR.h"

//Chip select
void IMUchipSelect(IMU* IMU) {
	HAL_GPIO_WritePin(IMU->CS_GPIO_Port, IMU->CS_GPIO_Pin, GPIO_PIN_RESET); //CS pin
}

//Chip release
void IMUchipRelease(IMU* IMU) {
	HAL_GPIO_WritePin(IMU->CS_GPIO_Port, IMU->CS_GPIO_Pin, GPIO_PIN_SET); //CS pin
}

//Read register from IMU
HAL_StatusTypeDef IMUread(IMU* IMU, uint8_t reg_addr, uint8_t rx_buffer, uint8_t num_bytes, uint8_t timeout) {
	uint8_t reg_buffer[0] = {reg_addr | 0x80};  // Set register for reading

	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	IMUchipSelect(IMU);
	HAL_SPI_Transmit(IMU->hspi, (uint8_t *)reg_buffer, 1, timeout);
	status = HAL_SPI_Receive(IMU->hspi, (uint8_t *)rx_buffer, num_bytes, timeout);
	IMUchipRelease(IMU);

	taskEXIT_CRITICAL();
	return status;
}

//Write register from IMU
HAL_StatusTypeDef IMUwrite(IMU* IMU, uint8_t tx_buffer, uint8_t num_bytes, uint8_t timeout) {
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	IMUchipSelect(IMU);
	status = HAL_SPI_Transmit(IMU->hspi, (uint8_t *)tx_buffer, num_bytes + 1, timeout);
	IMUchipRelease(IMU);

	taskEXIT_CRITICAL();
	return status;
}

//Initialize IMU
int IMUinit(SPI_HandleTypeDef* hspi, IMU* IMU, GPIO_TypeDef * CS_GPIO_Port, uint16_t CS_GPIO_Pin) {
	IMU->hspi = hspi;
	IMU->CS_GPIO_Port = CS_GPIO_Port;
	IMU->CS_GPIO_Pin = CS_GPIO_Pin;

	//Read WHO_AM_I register
	uint8_t rx_buffer;
	IMUread(IMU, WHO_AM_I_REG_ADDR, rx_buffer, 1, 1000);
	//Check if WHO_AM_I register is correct
	if (rx_buffer != WHO_AM_I_REG_VAL) {
		return -1;
	}

	//There may have to be startup commands here too? idk haven't tested
	return 0;
}

//Get acceleration from IMU
int IMUgetAccel(IMU* IMU, Accel* accel) {
	return 0;
}

//Get angular rate from IMU
int IMUgetAngRate(IMU* IMU, AngRate* AngRate) {
	return 0;
}

//Get temperature from IMU
int IMUgetTemp(IMU* IMU, float* temp) {
	return 0;
}

//Send command to IMU
int IMUsend() {
	//this might not be needed
	return 0;
}