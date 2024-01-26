/*
 * LSM6DSO32XTR.c
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#include "../inc/LSM6DSO32XTR.h"

//Temp convert
float IMU_tempConvert(uint8_t H_Byte, uint8_t L_Byte) {
    int16_t temp_raw = (H_Byte << 8) | L_Byte; // Combine the two bytes into one 16 bit number
	
    if (temp_raw & 0x8000) { // If the sign bit is set
        temp_raw = ~temp_raw + 1; // Calculate two's complement
        temp_raw = -temp_raw; // Convert to negative
    }

    float temp = temp_raw*SCALING_FACTOR_TEMP; // Convert to degrees C

    return temp;
}

//IMU convert
float IMU_accelConvert(uint8_t H_Byte, uint8_t L_Byte) {
	int16_t accel_raw = (H_Byte << 8) | L_Byte; // Combine the two bytes into one 16 bit number
	
	if (accel_raw & 0x8000) { // If the sign bit is set
		accel_raw = ~accel_raw + 1; // Calculate two's complement
		accel_raw = -accel_raw; // Convert to negative
	}

	float accel = accel_raw*SCALING_FACTOR_ACCEL; // Convert to g

	return accel;
}

//Gyro convert
float IMU_gyroConvert(uint8_t H_Byte, uint8_t L_Byte) {
	int16_t gyro_raw = (H_Byte << 8) | L_Byte; // Combine the two bytes into one 16 bit number
	
	if (gyro_raw & 0x8000) { // If the sign bit is set
		gyro_raw = ~gyro_raw + 1; // Calculate two's complement
		gyro_raw = -gyro_raw; // Convert to negative
	}

	float gyro = gyro_raw*SCALING_FACTOR_GYRO; // Convert to dps

	return gyro;
}

//Chip select
void IMU_chipSelect(IMU* IMU) {
	HAL_GPIO_WritePin(IMU->CS_GPIO_Port, IMU->CS_GPIO_Pin, GPIO_PIN_RESET); //CS pin
}

//Chip release
void IMU_chipRelease(IMU* IMU) {
	HAL_GPIO_WritePin(IMU->CS_GPIO_Port, IMU->CS_GPIO_Pin, GPIO_PIN_SET); //CS pin
}

//Read register from IMU
HAL_StatusTypeDef IMU_read(IMU* IMU, uint8_t reg_addr, uint8_t rx_buffer, uint8_t num_bytes) {
	uint8_t reg_buffer[0] = {reg_addr | 0x80};  // Set register for reading

	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	IMU_chipSelect(IMU);
	HAL_SPI_Transmit(IMU->hspi, (uint8_t *)reg_buffer, 1, IMU->SPI_TIMEOUT);
	status = HAL_SPI_Receive(IMU->hspi, (uint8_t *)rx_buffer, num_bytes, IMU->SPI_TIMEOUT);
	IMU_chipRelease(IMU);

	taskEXIT_CRITICAL();
	return status;
}

//Write register from IMU
HAL_StatusTypeDef IMU_write(IMU* IMU, uint8_t tx_buffer, uint8_t num_bytes) {
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	IMU_chipSelect(IMU);
	status = HAL_SPI_Transmit(IMU->hspi, (uint8_t *)tx_buffer, num_bytes + 1, IMU->SPI_TIMEOUT);
	IMU_chipRelease(IMU);

	taskEXIT_CRITICAL();
	return status;
}

//Initialize IMU
int IMU_init(SPI_HandleTypeDef* hspi, IMU* IMU, GPIO_TypeDef * CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint16_t SPI_TIMEOUT) {
	IMU->hspi = hspi;
	IMU->CS_GPIO_Port = CS_GPIO_Port;
	IMU->CS_GPIO_Pin = CS_GPIO_Pin;
	IMU->SPI_TIMEOUT = SPI_TIMEOUT;

	//Read WHO_AM_I register
	uint8_t rx_buffer;
	IMU_read(IMU, WHO_AM_I_REG_ADDR, rx_buffer, 1);
	//Check if WHO_AM_I register is correct
	if (rx_buffer != WHO_AM_I_REG_VAL) {
		return -1;
	}

	uint8_t cmd_buf[2];

	//Set up accelerometer
	cmd_buf[0] = CTRL1_XL;
	cmd_buf[1] = DEFAULT_CONF_ACCEL; // 01010000 208hz, + or - 4g range
	IMU_write(IMU, cmd_buf, 1);

	//Set up gyroscope
	cmd_buf[0] = CTRL2_G;
	cmd_buf[1] = DEFAULT_CONF_GYRO; // 01011100 208hz, + or - 2000dps range
	IMU_write(IMU, cmd_buf, 1);

	return 0;
}

//Get acceleration from IMU
int IMU_getAccel(IMU* IMU, Accel* accel) {
	uint8_t buf[2];

	IMU_read(IMU, OUTX_L_A, buf[0], 2);
	IMU_read(IMU, OUTX_H_A, buf[1], 2);
	accel->XL_x = IMU_accelConvert(buf[1], buf[0])+IMU->XL_x_offset;

	IMU_read(IMU, OUTY_L_A, buf[0], 2);
	IMU_read(IMU, OUTY_H_A, buf[1], 2);
	accel->XL_y = IMU_accelConvert(buf[1], buf[0])+IMU->XL_y_offset;

	IMU_read(IMU, OUTZ_L_A, buf[0], 2);
	IMU_read(IMU, OUTZ_H_A, buf[1], 2);
	accel->XL_z = IMU_accelConvert(buf[1], buf[0])+IMU->XL_z_offset;

	return 0;
}

//Get angular rate from IMU
int IMU_getAngRate(IMU* IMU, AngRate* AngRate) {
	uint8_t buf[2];

	IMU_read(IMU, OUTX_L_G, buf[0], 2);
	IMU_read(IMU, OUTX_H_G, buf[1], 2);
	AngRate->G_x = IMU_gyroConvert(buf[1], buf[0])+IMU->G_x_offset;

	IMU_read(IMU, OUTY_L_G, buf[0], 2);
	IMU_read(IMU, OUTY_H_G, buf[1], 2);
	AngRate->G_y = IMU_gyroConvert(buf[1], buf[0])+IMU->G_y_offset;

	IMU_read(IMU, OUTZ_L_G, buf[0], 2);
	IMU_read(IMU, OUTZ_H_G, buf[1], 2);
	AngRate->G_z = IMU_gyroConvert(buf[1], buf[0])+IMU->G_z_offset;
	return 0;
}

//Get temperature from IMU
int IMU_getTemp(IMU* IMU, float* temp) {
	uint8_t buf[2];

	IMU_read(IMU, OUT_TEMP_L, buf[0], 2);
	IMU_read(IMU, OUT_TEMP_H, buf[1], 2);
	*temp = IMU_tempConvert(buf[1], buf[0]);

	return 0;
}

//Send command to IMU
int IMU_send(IMU* IMU, uint8_t cmd, uint8_t value) {
	//idk why you wouldn't just use IMU_write but whatever
	uint8_t cmd_buf[2];
	cmd_buf[0] = cmd;
	cmd_buf[1] = value;
	IMU_write(IMU, cmd_buf, 1);

	return 0;
}