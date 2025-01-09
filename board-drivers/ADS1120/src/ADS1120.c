/*
 * ADS1120.c
 *
 *  Created on: November 24, 2024
 *      Author: felixfb
 */

#include "ADS1120.h"


double ADS_convertToMicrovolts(ADS1120 *ADS, uint8_t MSB, uint8_t LSB) {
	int16_t raw_value = (MSB << 8) | LSB;
	double stepSize = ADS->voltage_ref / pow(2, 15 + (ADS->gain >> 1));
	return stepSize * raw_value;
}

int ADS_init(ADS1120 *ADS, uint8_t mux, uint8_t gain, uint8_t rate, double reference_volts, float cold_junction_temp) {
	uint8_t resetcmd[1] = {0x06};
	ADS_write(ADS, resetcmd, 1);
	osDelay(1);

	ADS->voltage_ref = reference_volts * 1e6;
	// convert cjc temp to voltage using t type thermocouple polynomial //TODO

	int confStatus = ADS_configure(ADS, mux, gain, rate, 1);

	uint8_t startcmd[1] = {0x08};
	ADS_write(ADS, startcmd, 1);

	return confStatus;
}

HAL_StatusTypeDef ADS_write(ADS1120 *ADS, uint8_t *tx_buffer, uint8_t num_bytes) {
	taskENTER_CRITICAL();

	ADS_chipSelect(ADS);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(ADS->hspi, txbuffer, num_bytes, ADS->SPI_TIMEOUT);
	ADS_chipRelease(ADS);

	taskEXIT_CRITICAL();

	return status;
}

HAL_StatusTypeDef ADS_read(ADS1120 *ADS, uint8_t reg_addr, uint8_t *rx_buffer, uint8_t num_bytes) {
	taskENTER_CRITICAL();

	ADS_chipSelect(ADS);
	HAL_SPI_Transmit(ADS->hspi, &reg_addr, 1, ADS->SPI_TIMEOUT);
	HAL_StatusTypeDef status = HAL_SPI_Receive(ADS->hspi, rx_buffer, num_bytes, ADS->SPI_TIMEOUT);
	ADS_chipRelease(ADS);

	taskEXIT_CRITICAL();

	return status;
}

void ADS_chipSelect(ADS1120 *ADS) {
	HAL_GPIO_WritePin(ADS->CS_GPIO_Port, ADS->CS_GPIO_Pin, 0);
}

void ADS_chipRelease(ADS1120 *ADS) {
	HAL_GPIO_WritePin(ADS->CS_GPIO_Port, ADS->CS_GPIO_Pin, 1);
}

int ADS_configure(ADS1120 *ADS, uint8_t mux, uint8_t gain, uint8_t rate, int check) {
	uint8_t conf_byte_1 = mux | gain | ADS_PGA_ENABLED;
	uint8_t conf_byte_2 = rate | ADS_MODE_NORMAL | ADS_CONV_MODE_CONT | ADS_INTERNAL_TEMP_DISABLED | ADS_BURN_OUT_DISABLED;
	uint8_t conf_byte_3 = ADS_VOLT_REF_EXT_REF0 | ADS_FILTER_DISABLED | ADS_PSW_DISABLED | ADS_IDAC_DISABLED;
	uint8_t conf_byte_4 = ADS_l1MUX_DISABLED | ADS_l2MUX_DISABLED | ADS_DRDY_MODE_ONLY_DRDY;

	uint8_t txbuffer[5] = {0x43, conf_byte_1, conf_byte_2, conf_byte_3, conf_byte_4};

	ADS_write(ADS, txbuffer, 5);

	ADS->gain = gain;

	if(check) {
		uint8_t rxbuffer[4] = {0, 0, 0, 0};

		ADS_read(ADS, 0x23, rxbuffer, 4);

		return !(rxbuffer[0] == conf_byte_1 && rxbuffer[1] == conf_byte_2 && rxbuffer[2] == conf_byte_3 && rxbuffer[3] == conf_byte_4);
	}
	else {
		return 0;
	}
}
