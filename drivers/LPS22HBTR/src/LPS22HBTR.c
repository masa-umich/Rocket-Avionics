/*
 * LPS22HBTR.c
 *
 *  Created on: February 9, 2024
 *      Author: jackmh
 */

#include "../inc/LPS22HBTR.h"

//Pressure convert
float BAR_presConvert(uint8_t XL_Byte, uint8_t H_Byte, uint8_t L_Byte) {
//Cast to 24 bit int with 2s compliment
    int32_t pres_raw = (H_Byte << 16) | (L_Byte << 8) | (XL_Byte);
    if (pres_raw & 0x00800000) { // if sign bit is set
        pres_raw |= 0xFF000000; // extend the sign bit
    }
    float pres = pres_raw / BAR_PRES_SCALING_FACTOR; // Convert the 24 bit number to a pressure in hPa
    return pres;
}

//Altitude convert
float BAR_altConvert(uint8_t XL_Byte, uint8_t H_Byte, uint8_t L_Byte, float BAR_SEA_LEVEL_PRESS) {
    //Get the pressure first, then convert to altitude
    float pres = BAR_presConvert(XL_Byte, H_Byte, L_Byte);
    float alt = (1 - pow(pres / BAR_SEA_LEVEL_PRESS, 0.1903)) * 145366.45 * 0.3048; // Convert to meters above sea level
    return alt;
}

//Temperature convert
float BAR_tempConvert(uint8_t L_Byte, uint8_t H_Byte) {
    int16_t temp_raw = (int16_t)(H_Byte << 8) | L_Byte; // Combine the two bytes into one 16 bit number. This is already formatted for an int16_t

    float temp = temp_raw / BAR_TEMP_SCALING_FACTOR; // Convert to degrees C

    return temp;
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
    uint8_t buffer[12];

    //Set CTRL1_C register for 75Hz 
	buffer[0] = BAR_CTRL1_C;
	buffer[1] = BAR_DEFAULT_CONF_CTRL1_C;
	BAR_write(BAR, buffer, 1);

    //Set CTRL2_C register for sequential register update
    buffer[0] = BAR_CTRL2_C;
    buffer[1] = BAR_DEFAULT_CONF_CTRL2_C;
    BAR_write(BAR, buffer, 1);

    //Set CTRL3_C register with default values
    buffer[0] = BAR_CTRL3_C;
    buffer[1] = BAR_DEFAULT_CONF_CTRL3_C;
    BAR_write(BAR, buffer, 1);

	//Read WHO_AM_I register
	BAR_read(BAR, BAR_WHO_AM_I_REG_ADDR, buffer, 1);
	//Check if WHO_AM_I register is correct
	if (buffer[0] != BAR_WHO_AM_I_REG_VAL) {
		return -1;
	}
	return 0;
}

//Get pressure from barometer
int BAR_getPres(BAR* BAR, float* pres) {
    uint8_t buffer[3];

    //Read pressure data
    BAR_read(BAR, BAR_PRESS_OUT_XL, &buffer[0], 1);
    BAR_read(BAR, BAR_PRESS_OUT_L, &buffer[1], 1);
    BAR_read(BAR, BAR_PRESS_OUT_H, &buffer[2], 1);
    *pres = BAR_presConvert(buffer[2], buffer[1], buffer[0]);
    return 0;
}

//Get altitude from barometer
int BAR_getAlt(BAR* BAR, float* alt, float BAR_SEA_LEVEL_PRESS) {
    uint8_t buffer[3];

    //Read pressure data
    BAR_read(BAR, BAR_PRESS_OUT_XL, buffer, 3);
    *alt = BAR_altConvert(buffer[2], buffer[1], buffer[0], BAR_SEA_LEVEL_PRESS);
    return 0;
}

int BAR_getTemp(BAR* BAR, float* temp) {
    uint8_t buffer[2];

    //Read temperature data
    BAR_read(BAR, BAR_TEMP_OUT_L, &buffer[0], 1);
    BAR_read(BAR, BAR_TEMP_OUT_H, &buffer[1], 1);
    *temp = BAR_tempConvert(buffer[0], buffer[1]);
    return 0;
}

//Send command to barometer
int BAR_send(BAR* BAR, uint8_t cmd, uint8_t value) {
	return 0;
}
