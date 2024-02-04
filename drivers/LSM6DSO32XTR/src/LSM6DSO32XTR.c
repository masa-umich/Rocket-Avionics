/*
 * LSM6DSO32XTR.c
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#include "../inc/LSM6DSO32XTR.h"

//Temp convert
float IMU_tempConvert(uint8_t H_Byte, uint8_t L_Byte) {
    int16_t temp_raw = (int16_t)(H_Byte << 8) + L_Byte; // Combine the two bytes into one 16 bit number. This is already formatted for an int16_t

    float temp = temp_raw*SCALING_FACTOR_TEMP; // Convert to degrees C

    return temp;
}

//IMU convert
float IMU_accelConvert(uint8_t H_Byte, uint8_t L_Byte) {
	int16_t accel_raw = (int16_t)(H_Byte << 8) + L_Byte; // Combine the two bytes into one 16 bit number. This is already formatted for an int16_t

	float accel = (accel_raw*SCALING_FACTOR_ACCEL)*g; // Convert to m/s^2

	return accel;
}

//Gyro convert
float IMU_gyroConvert(uint8_t H_Byte, uint8_t L_Byte) {
	int16_t gyro_raw = (int16_t)(H_Byte << 8) + L_Byte; // Combine the two bytes into one 16 bit number. This is already formatted for an int16_t

	float gyro = gyro_raw*SCALING_FACTOR_GYRO; // Convert to dps

	return gyro;
}

//Chip select
void IMU_chipSelect(IMU* IMU) {
	HAL_GPIO_WritePin(IMU->CS_GPIO_Port, IMU->CS_GPIO_Pin, 0); //CS pin
}

//Chip release
void IMU_chipRelease(IMU* IMU) {
	HAL_GPIO_WritePin(IMU->CS_GPIO_Port, IMU->CS_GPIO_Pin, 1); //CS pin
}

//Read register from IMU
HAL_StatusTypeDef IMU_read(IMU* IMU, uint8_t reg_addr, uint8_t* rx_buffer, uint8_t num_bytes) {
	uint8_t reg_buffer[1] = {reg_addr | 0x80};  // Set register for reading

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
HAL_StatusTypeDef IMU_write(IMU* IMU, uint8_t* tx_buffer, uint8_t num_bytes) {
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	IMU_chipSelect(IMU);
	status = HAL_SPI_Transmit(IMU->hspi, (uint8_t *)tx_buffer, num_bytes + 1, IMU->SPI_TIMEOUT);
	IMU_chipRelease(IMU);

	taskEXIT_CRITICAL();
	return status;
}

//Initialize IMU
int IMU_init(IMU* IMU) {
	uint8_t buffer[12];

	//Main control register
	//This needs to be set first because this determines how
	//future register controls work
	buffer[0] = CTRL3_C;
	buffer[1] = DEFAULT_CONF_CTRL3_C;
	IMU_write(IMU, buffer, 1);

	//Read WHO_AM_I register
	IMU_read(IMU, WHO_AM_I_REG_ADDR, buffer, 1);
	//Check if WHO_AM_I register is correct
	if (buffer[0] != WHO_AM_I_REG_VAL) {
		return -1;
	}


	//Set up accelerometer
	buffer[0] = CTRL1_XL;
	buffer[1] = DEFAULT_CONF_ACCEL; // 01010000 208hz, + or - 4g range
	IMU_write(IMU, buffer, 1);

	//Set up gyroscope
	buffer[0] = CTRL2_G;
	buffer[1] = DEFAULT_CONF_GYRO; // 01011100 208hz, + or - 2000dps range
	IMU_write(IMU, buffer, 1);

	/*
	//Other default configs
	buffer[0] = CTRL4_C;
	buffer[1] = DEFAULT_CONF_CTRL4_C;
	IMU_write(IMU, buffer, 1);

	buffer[0] = CTRL5_C;
	buffer[1] = DEFAULT_CONF_CTRL5_C;
	IMU_write(IMU, buffer, 1);

	buffer[0] = CTRL6_C;
	buffer[1] = DEFAULT_CONF_CTRL6_C; 
	IMU_write(IMU, buffer, 1);

	buffer[0] = CTRL7_G;
	buffer[1] = DEFAULT_CONF_CTRL7_G;
	IMU_write(IMU, buffer, 1);

	buffer[0] = CTRL8_XL;
	buffer[1] = DEFAULT_CONF_CTRL8_XL;
	IMU_write(IMU, buffer, 1);

	buffer[0] = CTRL9_XL;
	buffer[1] = DEFAULT_CONF_CTRL9_XL;
	IMU_write(IMU, buffer, 1);

	buffer[0] = CTRL10_C;
	buffer[1] = DEFAULT_CONF_CTRL10_C;
	IMU_write(IMU, buffer, 1);
	*/

	return 0;
}

//Get acceleration from IMU
int IMU_getAccel(IMU* IMU, Accel* accel) {
	uint8_t buf[6];

	IMU_read(IMU, OUTX_L_A, buf, 6);
	//We are able to do a single read because when doing multiple reads, it will automatically increment the register address
	//This is good not only for code duplication, but also reduces overhead
	//The option to disable this is in the datasheet under CTRL3_C
	accel->XL_x = IMU_accelConvert(buf[1], buf[0]) + IMU->XL_x_offset;
	accel->XL_y = IMU_accelConvert(buf[3], buf[2]) + IMU->XL_y_offset;
	accel->XL_z = IMU_accelConvert(buf[5], buf[4]) + IMU->XL_z_offset;

	return 0;
}

//Get angular rate from IMU
int IMU_getAngRate(IMU* IMU, AngRate* AngRate) {
	uint8_t buf[6];

	IMU_read(IMU, OUTX_L_G, buf, 6);
	//Same as before, we can do a single read
	AngRate->G_x = IMU_gyroConvert(buf[1], buf[0])+IMU->G_x_offset;
	AngRate->G_y = IMU_gyroConvert(buf[3], buf[2])+IMU->G_y_offset;
	AngRate->G_z = IMU_gyroConvert(buf[5], buf[4])+IMU->G_z_offset;

	return 0;
}

//Get temperature from IMU
int IMU_getTemp(IMU* IMU, float* temp) {
	uint8_t buf[2];

	IMU_read(IMU, OUT_TEMP_L, buf, 2);

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
