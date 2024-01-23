/*
 * LSM6DSO32XTR.c
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#include "../inc/LSM6DSO32XTR.h"

//Temp convert
float IMUfloatConvert(uint8_t H_Byte, uint8_t L_Byte) {
    int16_t temp_raw = (H_Byte << 8) | L_Byte; // Combine the two bytes into one 16 bit number
    if (temp_raw & 0x8000) { // If the sign bit is set
        temp_raw = ~temp_raw + 1; // Calculate two's complement
        temp_raw = -temp_raw; // Convert to negative
    }
    float scale_factor = 0.00190734863; // 125/2^16 (operating range divided by ADC resolution)
    float temp = temp_raw * scale_factor; // Convert to degrees C
    return temp;
}

//Chip select
void IMUchipSelect(IMU* IMU) {
	HAL_GPIO_WritePin(IMU->CS_GPIO_Port, IMU->CS_GPIO_Pin, GPIO_PIN_RESET); //CS pin
}

//Chip release
void IMUchipRelease(IMU* IMU) {
	HAL_GPIO_WritePin(IMU->CS_GPIO_Port, IMU->CS_GPIO_Pin, GPIO_PIN_SET); //CS pin
}

//Read register from IMU
HAL_StatusTypeDef IMUread(IMU* IMU, uint8_t reg_addr, uint8_t rx_buffer, uint8_t num_bytes) {
	uint8_t reg_buffer[0] = {reg_addr | 0x80};  // Set register for reading

	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	IMUchipSelect(IMU);
	HAL_SPI_Transmit(IMU->hspi, (uint8_t *)reg_buffer, 1, IMU->SPI_TIMEOUT);
	status = HAL_SPI_Receive(IMU->hspi, (uint8_t *)rx_buffer, num_bytes, IMU->SPI_TIMEOUT);
	IMUchipRelease(IMU);

	taskEXIT_CRITICAL();
	return status;
}

//Write register from IMU
HAL_StatusTypeDef IMUwrite(IMU* IMU, uint8_t tx_buffer, uint8_t num_bytes) {
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	IMUchipSelect(IMU);
	status = HAL_SPI_Transmit(IMU->hspi, (uint8_t *)tx_buffer, num_bytes + 1, IMU->SPI_TIMEOUT);
	IMUchipRelease(IMU);

	taskEXIT_CRITICAL();
	return status;
}

//Initialize IMU
int IMUinit(SPI_HandleTypeDef* hspi, IMU* IMU, GPIO_TypeDef * CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint16_t SPI_TIMEOUT) {
	IMU->hspi = hspi;
	IMU->CS_GPIO_Port = CS_GPIO_Port;
	IMU->CS_GPIO_Pin = CS_GPIO_Pin;
	IMU->SPI_TIMEOUT = SPI_TIMEOUT;

	//Read WHO_AM_I register
	uint8_t rx_buffer;
	IMUread(IMU, WHO_AM_I_REG_ADDR, rx_buffer, 1);
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
	uint8_t buf[2];

	IMUread(IMU, OUT_TEMP_L, buf, 2);
	*temp = IMUfloatConvert(buf[1], buf[0]);

	return 0;
}

//Send command to IMU
int IMUsend() {
	//this might not be needed
	return 0;
}