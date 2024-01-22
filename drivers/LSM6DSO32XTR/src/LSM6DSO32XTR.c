/*
 * LSM6DSO32XTR.c
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#include "../inc/LSM6DSO32XTR.h"

#define WHO_AM_I_REG_ADDR (uint8_t) 0x0F //Who am I register address

//Initialize IMU
int IMUinit(SPI_HandleTypeDef* hspi1, int timeout) {
	//uint8_t who_am_i_read_addr;
	uint8_t who_am_i_read_addr = WHO_AM_I_REG_ADDR | 0x80;  // Set register for reading
	uint8_t who_am_i;  // Who am I value
	HAL_StatusTypeDef debug_status;  // Debug status
	
	//Make sure nothing happens to our SPI bus while we're reading and writing to it
	taskENTER_CRITICAL();
	
	//Send SPI message
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS pin
	debug_status = HAL_SPI_Transmit(hspi1, &who_am_i_read_addr, 1, timeout);
	if (debug_status != HAL_OK) { return 1; }
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS pin

	//Read SPI register
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS pin
	debug_status = HAL_SPI_Receive(hspi1, &who_am_i, 1, timeout);
	if (debug_status != HAL_OK) { return 1; }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS pin
	
	//End critical section
	taskEXIT_CRITICAL();

	//Check if who am I is correct
	if (who_am_i != 0x6C) {
		return 1;
	}
	
	return 0;
}

//Send command to IMU
int IMUsend() {
	return 0;
}

//Read register from IMU
int IMUread() {
	return 0;
}

//Write register from IMU
int IMUwrite() {
	return 0;
}

//Get acceleration from IMU
int IMUgetAccel() {
	return 0;
}

//Get angular rate from IMU
int IMUgetAngRate() {
	return 0;
}

//Get temperature from IMU
int IMUgetTemp() {
	return 0;
}
