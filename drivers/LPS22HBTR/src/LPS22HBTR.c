/*
 * LPS22HBTR.c
 *
 *  Created on: February 9, 2024
 *      Author: jackmh
 */

#include "LPS22HBTR.h"

//Pressure convert
float BAR_presConvert(uint8_t XL_Byte, uint8_t L_Byte, uint8_t H_Byte) {
    //Cast to 24 bit int with 2s compliment
    int32_t pres_raw = (H_Byte << 16) | (L_Byte << 8) | (XL_Byte);
    float pres = (float)(pres_raw / (float)BAR_PRES_SCALING_FACTOR); // Convert the 24 bit number to a pressure in hPa
    return pres;
}

//Altitude convert
float BAR_altConvert(uint8_t XL_Byte, uint8_t L_Byte, uint8_t H_Byte, float BAR_SEA_LEVEL_PRESS) {
    //Get the pressure first, then convert to altitude
    float pres = BAR_presConvert(XL_Byte, L_Byte, H_Byte);
    float alt = (1 - pow(pres / BAR_SEA_LEVEL_PRESS, 0.1903)) * 145366.45 * 0.3048; // Convert to meters above sea level
    return alt;
}

//Temperature convert
float BAR_tempConvert(uint8_t L_Byte, uint8_t H_Byte) {
    int16_t temp_raw = (int16_t)(H_Byte << 8) | L_Byte; // Combine the two bytes into one 16 bit number. This is already formatted for an int16_t

    float temp = (float)temp_raw / (float)BAR_TEMP_SCALING_FACTOR; // Convert to degrees C

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

//Initialize barometer with default settings
int BAR_init(BAR* BAR) {
    BAR_chipRelease(BAR); // Make sure the chip is not selected
	BAR_Reset(BAR); // Reset the barometer, built-in delay for reboot

    // Set default configs
    BAR_send(BAR, BAR_CTRL1_C, BAR_DEFAULT_CONF_CTRL1_C);
    BAR_send(BAR, BAR_CTRL2_C, BAR_DEFAULT_CONF_CTRL2_C);

    return BAR_whoami(BAR);
}

//Get pressure from barometer
int BAR_getPres(BAR* BAR, float* pres) {
    uint8_t buffer[3] = {0,0,0};
    //Wait for pressure data to be ready
    BAR_waitForPres(BAR); // Blocks until the pressure data is ready

    //Read pressure data
    BAR_read(BAR, BAR_PRESS_OUT_XL, buffer, 3);
    *pres = BAR_presConvert(buffer[0], buffer[1], buffer[2]);
    return 0;
}

//Get altitude from barometer
int BAR_getAlt(BAR* BAR, float* alt, float BAR_SEA_LEVEL_PRESS) {
    uint8_t buffer[3] = {0,0,0};

    //Wait for pressure data to be ready
    BAR_waitForPres(BAR); // Blocks until the pressure data is ready

    //Read pressure data
    BAR_read(BAR, BAR_PRESS_OUT_XL, buffer, 3);
    *alt = BAR_altConvert(buffer[0], buffer[1], buffer[2], BAR_SEA_LEVEL_PRESS);
    return 0;
}

int BAR_getTemp(BAR* BAR, float* temp) {
    uint8_t buffer[3] = {0,0};

    //Wait for temperature data to be ready
    BAR_waitForTemp(BAR); // Blocks until the temperature data is ready

    //Read temperature data
    BAR_read(BAR, BAR_TEMP_OUT_L, buffer, 2);
    *temp = BAR_tempConvert(buffer[0], buffer[1]);
    return 0;
}

//Send command to barometer
int BAR_send(BAR* BAR, uint8_t cmd, uint8_t value) {
	uint8_t buffer[2] = {0,0};
	//Send command
	buffer[0] = cmd;
	buffer[1] = value;
	BAR_write(BAR, buffer, 1);
	//Check that it wrote successfully
	BAR_read(BAR, cmd, &buffer[0], 1);
    if (buffer[0]!=value) {
        return 1; //Error
    }

	return 0;
}

//Return 0 is WHO_AM_I register can be read, 1 otherwise
int BAR_whoami(BAR* BAR) {
	uint8_t buffer = 0;
	//Read WHO_AM_I register
	BAR_read(BAR, BAR_WHO_AM_I_REG_ADDR, &buffer, 1);
	//Check if WHO_AM_I register is correct
	if (buffer != BAR_WHO_AM_I_REG_VAL) {
		return -1;
	}
	return 0;
}

//Waits / blocks for the pressure data to be ready
int BAR_waitForPres(BAR* BAR) {
    uint8_t status = 0;
    do {
    	BAR_read(BAR, BAR_STATUS_REG, &status, 1);
    } while (!(status & 0x2));
    return 0;
}

//Waits / blocks for the temperature data to be ready
int BAR_waitForTemp(BAR* BAR) {
    uint8_t status = 0;
    do {
    	BAR_read(BAR, BAR_STATUS_REG, &status, 1);
    } while (!(status & 0x1));
    return 0;
}

// Software and memory reset
int BAR_Reset(BAR* BAR) {
    BAR_send(BAR, BAR_CTRL2_C, BAR_SW_RESET);

    HAL_Delay(50); // Ensure the reset is complete
    return 0;
}
