/*
 * ADS1120.c
 *
 *  Created on: November 24, 2024
 *      Author: felixfb
 */

#include "ADS1120.h"


double ADS_convertToMicrovolts(ADS1120 *ADS, uint8_t MSB, uint8_t LSB) {
	int16_t raw_value = (MSB << 8) | LSB;
	double stepSize = 1e3 / (0x0001 << (4 + (ADS->gain >> 1))); // I got carried away optimizing this, I don't know why. Only works for a reference voltage of 2.048V
	return stepSize * raw_value;
}

float ADS_polyMicrovoltsToTemp(double microvolts) {
	if(microvolts < 0) {
		double temp = microvolts * ADS_POLY_INV_NEG_C1 +
				pow(microvolts, 2) * ADS_POLY_INV_NEG_C2 +
				pow(microvolts, 3) * ADS_POLY_INV_NEG_C3 +
				pow(microvolts, 4) * ADS_POLY_INV_NEG_C4 +
				pow(microvolts, 5) * ADS_POLY_INV_NEG_C5 +
				pow(microvolts, 6) * ADS_POLY_INV_NEG_C6 +
				pow(microvolts, 7) * ADS_POLY_INV_NEG_C7;
		return (float)temp;

	}
	else {
		double temp = microvolts * ADS_POLY_INV_POS_C1 +
				pow(microvolts, 2) * ADS_POLY_INV_POS_C2 +
				pow(microvolts, 3) * ADS_POLY_INV_POS_C3 +
				pow(microvolts, 4) * ADS_POLY_INV_POS_C4 +
				pow(microvolts, 5) * ADS_POLY_INV_POS_C5 +
				pow(microvolts, 6) * ADS_POLY_INV_POS_C6;
		return (float)temp;
	}
}

double ADS_polyTempToMicrovolts(float temp) {
	if(temp < 0) {
		return temp * ADS_POLY_NEG_C1 +
				pow(temp, 2) * ADS_POLY_NEG_C2 +
				pow(temp, 3) * ADS_POLY_NEG_C3 +
				pow(temp, 4) * ADS_POLY_NEG_C4 +
				pow(temp, 5) * ADS_POLY_NEG_C5 +
				pow(temp, 6) * ADS_POLY_NEG_C6 +
				pow(temp, 7) * ADS_POLY_NEG_C7 +
				pow(temp, 8) * ADS_POLY_NEG_C8 +
				pow(temp, 9) * ADS_POLY_NEG_C9 +
				pow(temp, 10) * ADS_POLY_NEG_C10 +
				pow(temp, 11) * ADS_POLY_NEG_C11 +
				pow(temp, 12) * ADS_POLY_NEG_C12 +
				pow(temp, 13) * ADS_POLY_NEG_C13 +
				pow(temp, 14) * ADS_POLY_NEG_C14;
	}
	else {
		return temp * ADS_POLY_POS_C1 +
				pow(temp, 2) * ADS_POLY_POS_C2 +
				pow(temp, 3) * ADS_POLY_POS_C3 +
				pow(temp, 4) * ADS_POLY_POS_C4 +
				pow(temp, 5) * ADS_POLY_POS_C5 +
				pow(temp, 6) * ADS_POLY_POS_C6 +
				pow(temp, 7) * ADS_POLY_POS_C7 +
				pow(temp, 8) * ADS_POLY_POS_C8;
	}
}

int ADS_init(ADS1120 *ADS, uint8_t mux, uint8_t gain, uint8_t rate, float cold_junction_temp) {
	ADS->mux = mux;
	ADS->gain = gain;
	ADS->rate = rate;
	ADS->cold_junction_voltage = ADS_polyTempToMicrovolts(cold_junction_temp);

	return ADS_configureChip(ADS, 1);
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
	uint8_t conf_byte_3 = ADS_VOLT_REF_INT | ADS_FILTER_DISABLED | ADS_PSW_DISABLED | ADS_IDAC_DISABLED;
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

int ADS_configureChip(ADS1120 *ADS, int check) {
	uint8_t conf_byte_3 = ADS_VOLT_REF_INT | ADS_FILTER_DISABLED | ADS_PSW_DISABLED | ADS_IDAC_DISABLED;
	uint8_t conf_byte_4 = ADS_l1MUX_DISABLED | ADS_l2MUX_DISABLED | ADS_DRDY_MODE_ONLY_DRDY;

	uint8_t txbuffer[3] = {0x49, conf_byte_3, conf_byte_4};

	ADS_write(ADS, txbuffer, 3);

	if(check) {
		uint8_t rxbuffer[2] = {0, 0};

		ADS_read(ADS, 0x29, rxbuffer, 2);

		return !(rxbuffer[2] == conf_byte_3 && rxbuffer[3] == conf_byte_4);
	}
	else {
		return 0;
	}
}

int ADS_switchConf(ADS1120 *ADS) {
	uint8_t conf_byte_1 = ADS->mux | ADS->gain | ADS_PGA_ENABLED;
	uint8_t conf_byte_2 = ADS->rate | ADS_MODE_NORMAL | ADS_CONV_MODE_SING | ADS_INTERNAL_TEMP_DISABLED | ADS_BURN_OUT_DISABLED;

	uint8_t txbuffer[3] = {0x41, conf_byte_1, conf_byte_2};

	ADS_write(ADS, txbuffer, 3);
}
