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
HAL_StatusTypeDef MS5611_read(MS5611* BAR, uint8_t* rx_buffer, uint8_t num_bytes) {
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();
	
	MS5611_chipSelect(BAR);
	status = HAL_SPI_Receive(BAR->hspi, (uint8_t *)rx_buffer, num_bytes, BAR->SPI_TIMEOUT);
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
	if (MS5611_write(BAR, &reg, 1) != HAL_OK) {
		return 1;
	} else {
		return 0;
	}
}

int MS5611_recieve(MS5611* BAR, uint8_t rx_num_bytes, uint8_t* rx_buffer) {
	if (MS5611_read(BAR, rx_buffer, rx_num_bytes) != HAL_OK) {
		return 1;
	}
	return 0;
}

// Software and memory reset
int MS5611_Reset(MS5611* BAR) {
	if (!MS5611_send(BAR, MS5611_RESET)) {
		vTaskDelay(5); // Time for reset
    	return 0;
	}
	return 1;
}

// Read the programmable read only memory
// Note: prom_buffer must be of size 6
int MS5611_readPROM(MS5611* BAR, MS5611_PROM_t* prom_buffer) {
	for (int i = 0; i < 6; i++) {
		if (MS5611_send(BAR, MS5611_PROM + i)) {
			return 1;
		}
		uint8_t rx_buffer[2] = {0};
		if (MS5611_recieve(BAR, 2, rx_buffer)) {
			return 1;
		} else {
			prom_buffer->bytes[i] = (rx_buffer[0] << 8) | rx_buffer[1];
		}
	}
	return 0;
}

// Pressure convert
// OSR is the "Over Sampling Rate" which determines the resolution of the pressure and temperature
int MS5611_presConvert(MS5611* BAR, uint32_t* pres_raw, OSR osr) {
	uint8_t cmd = 0x00;
	switch (osr) {
		case OSR_256:
			cmd = MS5611_D1_OSR_256;
		case OSR_512:
			cmd = MS5611_D1_OSR_512;
		case OSR_1024:
			cmd = MS5611_D1_OSR_1024;
		case OSR_2048:
			cmd = MS5611_D1_OSR_2048;
		case OSR_4096:
			cmd = MS5611_D1_OSR_4096;
		default:
			return 1; // invalid OSR
	}
	if (cmd == 0x00) { return 1; } // invalid cmd
	MS5611_send(BAR, cmd);
	uint8_t rx_buffer[3] = {0};
	if (MS5611_recieve(BAR, 3, rx_buffer)) {
		return 1;
	}
	*pres_raw = (rx_buffer[0] << 16) | (rx_buffer[1] << 8) | (rx_buffer[2]);
	return 0;
}

//Altitude convert
int MS5611_tempConvert(MS5611* BAR, uint32_t* temp_raw, OSR osr) {
	uint8_t cmd = 0x00;
	switch (osr) {
		case OSR_256:
			cmd = MS5611_D2_OSR_256;
		case OSR_512:
			cmd = MS5611_D2_OSR_512;
		case OSR_1024:
			cmd = MS5611_D2_OSR_1024;
		case OSR_2048:
			cmd = MS5611_D2_OSR_2048;
		case OSR_4096:
			cmd = MS5611_D2_OSR_4096;
		default:
			return 1; // invalid OSR
	}
	if (cmd == 0x00) { return 1; } // invalid cmd
	MS5611_send(BAR, cmd);
	uint8_t rx_buffer[3] = {0};
	if (MS5611_recieve(BAR, 3, rx_buffer)) {
		return 1;
	}
	*temp_raw = (rx_buffer[0] << 16) | (rx_buffer[1] << 8) | (rx_buffer[2]);
	return 0;
}

int MS5611_compensateTemp(float* pres, uint32_t pres_raw, uint32_t temp_raw, MS5611_PROM_t* prom) {
	int32_t dT = temp_raw - (uint32_t)(prom->constants.C5 * pow(2,8));
	//int32_t temp = 2000 + dT * prom->constants.C6 / pow(2,23); //unused for now

	int64_t offset = prom->constants.C2 * pow(2,16) + (prom->constants.C4 * dT) / pow(2,7);
	int64_t sensitivity = prom->constants.C1 * pow(2,15) + (prom->constants.C3 * dT) / pow(2,8);

	*pres = ((pres_raw * sensitivity) / (pow(2,21) - offset)) / pow(2,15);
	return 0;
}

//Get pressure from barometer
int MS5611_getPres(MS5611* BAR, float* pres, OSR osr) {
	MS5611_PROM_t prom = {};
	if (MS5611_readPROM(BAR, &prom) == 1) { return 1; }
	uint32_t pres_raw;
	uint32_t temp_raw;
	if (MS5611_presConvert(BAR, &pres_raw, osr) == 1) { return 1; }
	if (MS5611_tempConvert(BAR, &temp_raw, osr) == 1) { return 1; }
	if (MS5611_compensateTemp(pres, pres_raw, temp_raw, &prom) == 1) { return 1; }
	return 0;
}

//Get rough altitude based on pressure
int MS5611_getAlt(MS5611* BAR, float* alt, float BAR_SEA_LEVEL_PRESS) {
	return 0;
}
