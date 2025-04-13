/*
 * MS5611.c
 *
 *  Created on: January 12, 2025
 *      Author: jackmh
 */

#include "MS5611.h"

//Chip select
void MS5611_chipSelect(MS5611* BAR) {
	HAL_GPIO_WritePin(BAR->CS_GPIO_Port, BAR->CS_GPIO_Pin, RESET); //Low is selected
}

//Chip release
void MS5611_chipRelease(MS5611* BAR) {
	HAL_GPIO_WritePin(BAR->CS_GPIO_Port, BAR->CS_GPIO_Pin, SET); //High is released
}

//Transmit register from barometer
HAL_StatusTypeDef MS5611_transmit(MS5611* BAR, uint8_t* tx_buffer, uint8_t num_bytes) {
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	MS5611_chipSelect(BAR);
	status = HAL_SPI_Transmit(BAR->hspi, tx_buffer, 1, BAR->SPI_TIMEOUT);
	MS5611_chipRelease(BAR);

	taskEXIT_CRITICAL();

	return status;
}

//Receive register from barometer
HAL_StatusTypeDef MS5611_receive(MS5611* BAR, uint8_t* rx_buffer, uint8_t num_bytes) {
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	MS5611_chipSelect(BAR);
	status = HAL_SPI_Receive(BAR->hspi, rx_buffer, num_bytes, BAR->SPI_TIMEOUT);
	MS5611_chipRelease(BAR);

	taskEXIT_CRITICAL();

	return status;
}

// Abstraction for reading a register
// Note: num_bytes is the combined size of tx_buffer and rx_buffer
int MS5611_read(MS5611* BAR, uint8_t* tx_buffer, uint8_t* rx_buffer, uint8_t num_bytes) {
	HAL_StatusTypeDef status;

	taskENTER_CRITICAL();

	MS5611_chipSelect(BAR);
	status = HAL_SPI_TransmitReceive(BAR->hspi, tx_buffer, rx_buffer, num_bytes, BAR->SPI_TIMEOUT);
	MS5611_chipRelease(BAR);

	taskEXIT_CRITICAL();

	if (status != HAL_OK) {
		return 1;
	} else {
		return 0;
	}
}

// Abstraction for sending a single register
int MS5611_write(MS5611* BAR, uint8_t reg) {
	if (MS5611_transmit(BAR, &reg, 1) != HAL_OK) {
		return 1;
	}
	return 0;
}

// Software and memory reset
int MS5611_Reset(MS5611* BAR) {
	if (!MS5611_write(BAR, MS5611_RESET)) {
		vTaskDelay(5); // Time for reset
    	return 0;
	}
	return 1;
}

// Read the programmable read only memory
// Note: prom_buffer must be of size 6
int MS5611_readPROM(MS5611* BAR, MS5611_PROM_t* prom_buffer) {
	uint8_t tx_buffer[3] = {0};
	uint8_t rx_buffer[3] = {0};
	for (int i = 0; i < 6; i++) {
		tx_buffer[0] = MS5611_PROM + 2 + (i*2); // PROM is 2 bytes long and starts at 0xA2
		if (MS5611_read(BAR, tx_buffer, rx_buffer, 3) == 1) {
			return 1;
		} else {
			// for some reason the first byte we read back is always 0xFE
			// so we try to read three and use the last 2 instead
			prom_buffer->bytes[i] = (rx_buffer[1] << 8) | rx_buffer[2];
		}
	}
	return 0;
}

int MS5611_readADC(MS5611 *BAR, uint32_t *result) {
	uint8_t tx_buffer[3] = { 0 };
	uint8_t rx_buffer[3] = { 0 };
	tx_buffer[0] = MS5611_ADC;
	if (MS5611_read(BAR, tx_buffer, rx_buffer, 4) == 1) {
		return 1;
	} else {
		// for some reason the first byte we read back is always 0xFE
		// so we try to read four and use the last three instead
		*result = (rx_buffer[1] << 16) | (rx_buffer[2] << 8) | rx_buffer[3];
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
			break;
		case OSR_512:
			cmd = MS5611_D1_OSR_512;
			break;
		case OSR_1024:
			cmd = MS5611_D1_OSR_1024;
			break;
		case OSR_2048:
			cmd = MS5611_D1_OSR_2048;
			break;
		case OSR_4096:
			cmd = MS5611_D1_OSR_4096;
			break;
		default:
			return 1; // invalid OSR
	}
	if (cmd == 0x00) { return 1; } // invalid cmd
	MS5611_write(BAR, cmd); // write the command
	vTaskDelay(10); // 10ms conversion time
	if (MS5611_readADC(BAR, pres_raw)) {
		return 1;
	}
	return 0;
}

//Altitude convert
int MS5611_tempConvert(MS5611* BAR, uint32_t* temp_raw, OSR osr) {
	uint8_t cmd = 0x00;
	switch (osr) {
		case OSR_256:
			cmd = MS5611_D2_OSR_256;
			break;
		case OSR_512:
			cmd = MS5611_D2_OSR_512;
			break;
		case OSR_1024:
			cmd = MS5611_D2_OSR_1024;
			break;
		case OSR_2048:
			cmd = MS5611_D2_OSR_2048;
			break;
		case OSR_4096:
			cmd = MS5611_D2_OSR_4096;
			break;
		default:
			return 1; // invalid OSR
	}
	if (cmd == 0x00) { return 1; } // invalid cmd
	MS5611_write(BAR, cmd); // write the command
	vTaskDelay(10); // 10ms conversion time
	if (MS5611_readADC(BAR, temp_raw)) {
		return 1;
	}
	return 0;
}

int MS5611_compensateTemp(float* pres, uint32_t pres_raw, uint32_t temp_raw, MS5611_PROM_t* prom) {
	volatile int32_t dT, P;
	volatile int64_t OFF, SENS;
	volatile int64_t partialA, partialB;

	dT = temp_raw - ((prom->constants.C5) * (0x1<<8));                      //D2-C5*2^8
	//TEMP = 2000 + ((int64_t)dT * ((int64_t)prom->constants.C6))/(0x1<<23);//2000 + dT*C6/2^23
	partialA = ((int64_t)prom->constants.C4) * (int64_t)dT;
	OFF = ((int64_t)prom->constants.C2) * (0x1<<16) + (partialA)/(0x1<<7);  //C2*2^16 + C4*dT/2^7
	partialB = ((int64_t)prom->constants.C3) * (int64_t)dT;
	SENS = ((int64_t)prom->constants.C1) * (0x1<<15) + (partialB)/(0x1<<8); //C1*2^15 + C3*dT/2^8

	P = (((((int64_t)pres_raw)*SENS)/(0x1<<21)-OFF))/(0x1<<15);             //(D1*SENS/2^21-OFF)/2^15
	*pres = (float)(P * 0.01); // millibar
	return 0;
}

//Get pressure from barometer
int MS5611_getPres(MS5611* BAR, float* pres, MS5611_PROM_t* prom, OSR osr) {
	uint32_t pres_raw = 0;
	uint32_t temp_raw = 0;
	if (MS5611_presConvert(BAR, &pres_raw, osr) == 1) { return 1; }
	if (MS5611_tempConvert(BAR, &temp_raw, osr) == 1) { return 1; }
	if (MS5611_compensateTemp(pres, pres_raw, temp_raw, prom) == 1) { return 1; }
	return 0;
}

//Get rough altitude based on pressure
int MS5611_getAlt(MS5611* BAR, float* alt, MS5611_PROM_t* prom, OSR osr) {
	float pres = 0.0;
	uint32_t pres_raw = 0;
	uint32_t temp_raw = 0;
	if (MS5611_presConvert(BAR, &pres_raw, osr) == 1) { return 1; }
	if (MS5611_tempConvert(BAR, &temp_raw, osr) == 1) { return 1; }
	if (MS5611_compensateTemp(&pres, pres_raw, temp_raw, prom) == 1) { return 1; }
	*alt =  (1 - pow((pres/1013.25), 0.190284)) * 145366.45; // feet conversion
	return 0;
}