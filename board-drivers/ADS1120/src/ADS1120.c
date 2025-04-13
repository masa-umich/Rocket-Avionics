/*
 * ADS1120.c
 *
 *  Created on: November 24, 2024
 *      Author: Felix Foreman-Braunschweig
 */

#include "ADS1120.h"
//#include "semphr.h"

extern uint64_t getTimestamp(void);

void delay_us(uint32_t us) { // I found this online - why is it so difficult to delay less than a millisecond?
    uint32_t start = SysTick->VAL;
    uint32_t cycles = us * (SystemCoreClock / 1000000);

    if(start > cycles) {
    	while(SysTick->VAL > start - cycles && SysTick->VAL <= start);
    }
    else {
    	uint32_t end = ((SystemCoreClock/1000) - 1) - (cycles - start);
    	while(SysTick->VAL <= start || SysTick->VAL > end);
    }
}

double ADS_convertRawToMicrovolts(ADS_TC_t *ADS, int16_t raw) {
	//double stepSize = 1e3 / (0x0001 << (4 + (ADS->gain >> 1))); // I got carried away optimizing this, I don't know why. Only works for a reference voltage of 2.048V
	return ADS->step_size * raw;
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

void ADSTC_chipSelect(ADS_TC_t *ADS) {
	HAL_GPIO_WritePin(ADS->CS_GPIO_Port, ADS->CS_GPIO_Pin, 0);
}

void ADSTC_chipRelease(ADS_TC_t *ADS) {
	HAL_GPIO_WritePin(ADS->CS_GPIO_Port, ADS->CS_GPIO_Pin, 1);
}

void ADS_configTC(ADS_TC_t *ADSTC, SPI_HandleTypeDef* hspi, GPIO_TypeDef *DOUT_GPIO_Port, uint16_t DOUT_GPIO_Pin, uint16_t SPI_TIMEOUT, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint8_t mux, uint8_t gain, uint8_t rate) {
	ADSTC->hspi = hspi;
	ADSTC->DOUT_GPIO_Port = DOUT_GPIO_Port;
	ADSTC->DOUT_GPIO_Pin = DOUT_GPIO_Pin;
	ADSTC->SPI_TIMEOUT = SPI_TIMEOUT;
	ADSTC->CS_GPIO_Port = CS_GPIO_Port;
	ADSTC->CS_GPIO_Pin = CS_GPIO_Pin;
	ADSTC->mux = mux;
	ADSTC->gain = gain;
	ADSTC->rate = rate;
	ADSTC->step_size = (double)(1e3 / (0x0001 << (4 + (gain >> 1))));
	ADSTC->current_raw = 0;
	ADSTC->reading_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(ADSTC->reading_semaphore);

	ADSTC_chipRelease(ADSTC); // In case something weird happened
}

/*int ADS_init(ADS1120 *ADS, uint8_t mux, uint8_t gain, uint8_t rate, float cold_junction_temp) {
	ADS->mux = mux;
	ADS->gain = gain;
	ADS->rate = rate;
	ADS->cold_junction_voltage = ADS_polyTempToMicrovolts(cold_junction_temp);

	return ADS_configureChip(ADS, 1);
}*/

int ADS_init(ADS_Main_t *ADSMain, ADS_TC_t *TCs, uint8_t num_TCs) {
	ADSMain->TCs = TCs;
	ADSMain->TC_count = num_TCs;
	ADSMain->raw_temp = 0;
	ADSMain->last_temp = 0;
	ADSMain->temp_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(ADSMain->temp_semaphore);

	ADSMain->tc_task = NULL;
	BaseType_t state = xTaskCreate(vTCTask, "Thermocouple measurement task", 1024, (void *)ADSMain, TC_TASK_PRIORITY, &(ADSMain->tc_task)); //TODO figure out correct stack size
	return state != pdPASS;
}

// returns 0 if nothing should be done, returns 1 if dout should be checked, returns 2 if reading should be taken and moved on, basically a timeout
uint8_t ADS_timerStatus(ADS_Chip *ADS) {
	uint32_t delta_time = HAL_GetTick() - ADS->last_tick;
	uint32_t total_time = HAL_GetTick() - ADS->timer_start;
	uint8_t datarate = 0x00;
	if(ADS->current_mux == 2) {
		datarate = ADS_INTERNAL_TEMP_RATE;
	}
	else {
		datarate = ADS->muxes[ADS->current_mux]->rate;
	}
	uint8_t rate_delay = 3;
	if(datarate == ADS_DATA_RATE_20) {
		rate_delay = 65;
	}
	else if(datarate == ADS_DATA_RATE_45) {
		rate_delay = 30;
	}
	else if(datarate == ADS_DATA_RATE_90) {
		rate_delay = 20;
	}
	else if(datarate == ADS_DATA_RATE_175) {
		rate_delay = 15;
	}
	else if(datarate == ADS_DATA_RATE_330) {
		rate_delay = 10;
	}
	else if(datarate == ADS_DATA_RATE_600) {
		rate_delay = 5;
	}

	if(total_time >= rate_delay + 10) {
		return 2;
	}
	else if(total_time >= rate_delay) {
		return delta_time > 0;
	}
	return 0;
}

void vTCTask(void *pvParameters) {
	ADS_Main_t *ADS_main = (ADS_Main_t *)pvParameters;
	//loop through each tc struct, if its a new chip create a new chip struct, if its a existing chip but a new mux, add the new mux to the chip struct.
	//whenever you make a new chip struct, initialize all the spi fields and mux_count = 0 or 1
	size_t chip_count = 1;
	for(int s = 0;s < ADS_main->TC_count - 1;s++) {
		for(int ss = s+1; ss < ADS_main->TC_count;ss++) {
			if(ADS_main->TCs[s].hspi == ADS_main->TCs[ss].hspi && ADS_main->TCs[s].CS_GPIO_Port == ADS_main->TCs[ss].CS_GPIO_Port && ADS_main->TCs[s].CS_GPIO_Pin == ADS_main->TCs[ss].CS_GPIO_Pin) {
				break;
			}
			if(ss == ADS_main->TC_count - 1) {
				chip_count++;
			}
		}
	}
	ADS_Chip connected_chips[chip_count];
	connected_chips[0].hspi = ADS_main->TCs[0].hspi;
	connected_chips[0].DOUT_GPIO_Port = ADS_main->TCs[0].DOUT_GPIO_Port;
	connected_chips[0].DOUT_GPIO_Pin = ADS_main->TCs[0].DOUT_GPIO_Pin;
	connected_chips[0].SPI_TIMEOUT = ADS_main->TCs[0].SPI_TIMEOUT;
	connected_chips[0].CS_GPIO_Port = ADS_main->TCs[0].CS_GPIO_Port;
	connected_chips[0].CS_GPIO_Pin = ADS_main->TCs[0].CS_GPIO_Pin;
	connected_chips[0].muxes[0] = ADS_main->TCs;
	connected_chips[0].mux_count = 1;
	connected_chips[0].last_state = 0;

	uint8_t chip_index = 1;

	for(int s = 1;s < ADS_main->TC_count;s++) {
		uint8_t duplicate = 0;
		for(int ss = 0; ss < s;ss++) {
			if(ADS_main->TCs[s].hspi == ADS_main->TCs[ss].hspi && ADS_main->TCs[s].CS_GPIO_Port == ADS_main->TCs[ss].CS_GPIO_Port && ADS_main->TCs[s].CS_GPIO_Pin == ADS_main->TCs[ss].CS_GPIO_Pin) {
				duplicate = 1;
				break;
			}
		}
		if(!duplicate) {
			connected_chips[chip_index].hspi = ADS_main->TCs[s].hspi;
			connected_chips[chip_index].DOUT_GPIO_Port = ADS_main->TCs[s].DOUT_GPIO_Port;
			connected_chips[chip_index].DOUT_GPIO_Pin = ADS_main->TCs[s].DOUT_GPIO_Pin;
			connected_chips[chip_index].SPI_TIMEOUT = ADS_main->TCs[s].SPI_TIMEOUT;
			connected_chips[chip_index].CS_GPIO_Port = ADS_main->TCs[s].CS_GPIO_Port;
			connected_chips[chip_index].CS_GPIO_Pin = ADS_main->TCs[s].CS_GPIO_Pin;
			connected_chips[chip_index].muxes[0] = ADS_main->TCs + s;
			connected_chips[chip_index].mux_count = 1;
			connected_chips[chip_index].last_state = 0;
			chip_index++;
		}
		else {
			for(int c_index = 0;c_index < chip_index;c_index++) {
				if(ADS_main->TCs[s].hspi == connected_chips[c_index].hspi && ADS_main->TCs[s].CS_GPIO_Port == connected_chips[c_index].CS_GPIO_Port && ADS_main->TCs[s].CS_GPIO_Pin == connected_chips[c_index].CS_GPIO_Pin) {
					connected_chips[c_index].muxes[1] = ADS_main->TCs + s;
					connected_chips[c_index].mux_count = 2;
					break;
				}
			}
		}
	}

	// connected_chips should be populated with all used chips, and their muxes, and the first chip should be used to get internal temperature

	//reset
	for(int k = 0;k < chip_count;k++) {
		uint8_t reset_cmd[1] = {0x07};
		ADS_write(&(connected_chips[k]), reset_cmd, 1);

		/*ADS_chipSelect(&(connected_chips[k]));
		delay_us(50);
		taskENTER_CRITICAL();
		HAL_SPI_Transmit(connected_chips[k].hspi, reset_cmd, 1, connected_chips[k].SPI_TIMEOUT);
		taskEXIT_CRITICAL();
		delay_us(25);
		ADS_chipRelease(&(connected_chips[k]));*/


	}
	vTaskDelay(1);
	//config each chip with first mux
	for(int k = 0;k < chip_count;k++) {
		if(k == 0) {
			ADS_configure(&(connected_chips[k]), 2, 0);
			connected_chips[k].current_mux = 2;
		}
		else {
			ADS_configure(&(connected_chips[k]), 0, 0);
			connected_chips[k].current_mux = 0;
		}
		connected_chips[k].last_tick = HAL_GetTick();
		connected_chips[k].timer_start = HAL_GetTick();
	}

	for(;;) {
		for(int k = 0;k < chip_count;k++) {
			// update last_tick after timer returns 1 and dout is checked
			// update timer_start and last_tick after mux is changed

			// also if this is the first chip, get internal temp on timer
			
			uint8_t read_and_switch = 0;
			uint8_t timer = ADS_timerStatus(&(connected_chips[k]));
			if(timer == 1) {
				ADS_chipSelect(&(connected_chips[k]));
				delay_us(1);
				if(!HAL_GPIO_ReadPin(connected_chips[k].DOUT_GPIO_Port, connected_chips[k].DOUT_GPIO_Pin)) {
					read_and_switch = 1;
				}
				else {
					ADS_chipRelease(&(connected_chips[k]));
					connected_chips[k].last_tick = HAL_GetTick();
				}
			}

			if(timer == 2 || read_and_switch) {
				uint8_t next_index = connected_chips[k].current_mux + 1;
				uint8_t conf_byte_1 = 0x00;
				uint8_t conf_byte_2 = 0x00;

				if(k == 0 && HAL_GetTick() - ADS_main->last_temp > ADS_INTERNAL_TEMP_DELAY && connected_chips[k].current_mux != 2) {
					next_index = 2;
					conf_byte_1 = 0x00 | 0x00 | ADS_PGA_ENABLED;
					conf_byte_2 = ADS_INTERNAL_TEMP_RATE | ADS_MODE_NORMAL | ADS_CONV_MODE_SING | ADS_INTERNAL_TEMP_ENABLED | ADS_BURN_OUT_DISABLED;
				}
				else {
					if(next_index >= connected_chips[k].mux_count) {
						next_index = 0;
					}

					conf_byte_1 = connected_chips[k].muxes[next_index]->mux | connected_chips[k].muxes[next_index]->gain | ADS_PGA_ENABLED;
					conf_byte_2 = connected_chips[k].muxes[next_index]->rate | ADS_MODE_NORMAL | ADS_CONV_MODE_SING | ADS_INTERNAL_TEMP_DISABLED | ADS_BURN_OUT_DISABLED;
				}

				ADS_chipSelect(&(connected_chips[k]));
				delay_us(50);

				uint8_t MSB = 0x00;
				uint8_t LSB = 0x00;

				if(read_and_switch) { // normal reading, clock out reading and replace conf in one step
					uint8_t txbuffer[4] = {0x00, 0x41, conf_byte_1, conf_byte_2};
					uint8_t rxbuffer[4] = {0x00, 0x00, 0x00, 0x00};
					//uint8_t txbuffer[4] = {0x00, 0x00, 0x00, 0x00};
					//uint8_t rxbuffer[4] = {0x00, 0x00, 0x00, 0x00};

					taskENTER_CRITICAL();
					HAL_SPI_TransmitReceive(connected_chips[k].hspi, txbuffer, rxbuffer, 4, connected_chips[k].SPI_TIMEOUT);
					taskEXIT_CRITICAL();

					MSB = rxbuffer[0];
					LSB = rxbuffer[1];

					/*delay_us(1);

					uint8_t conftxbuffer[3] = {0x41, conf_byte_1, conf_byte_2};

					taskENTER_CRITICAL();
					HAL_SPI_Transmit(connected_chips[k].hspi, conftxbuffer, 3, connected_chips[k].SPI_TIMEOUT);
					taskEXIT_CRITICAL();*/

				}
				else { // timeout reading, use RDATA command and then replace conf
					/*
					RDATA command with 16 extra clock cycles?
					delay 1us
					write conf
					 */
					uint8_t read_txbuffer[3] = {0x10, 0x00, 0x00};
					uint8_t rxbuffer[3] = {0x00, 0x00, 0x00};

					taskENTER_CRITICAL();
					HAL_SPI_TransmitReceive(connected_chips[k].hspi, read_txbuffer, rxbuffer, 3, connected_chips[k].SPI_TIMEOUT);
					taskEXIT_CRITICAL();

					delay_us(1);

					uint8_t txbuffer[3] = {0x41, conf_byte_1, conf_byte_2};

					taskENTER_CRITICAL();
					HAL_SPI_Transmit(connected_chips[k].hspi, txbuffer, 3, connected_chips[k].SPI_TIMEOUT);
					taskEXIT_CRITICAL();

					MSB = rxbuffer[1];
					LSB = rxbuffer[2];
				}

				uint8_t startcmd[1] = {0x08};

				taskENTER_CRITICAL();
				HAL_SPI_Transmit(connected_chips[k].hspi, startcmd, 1, connected_chips[k].SPI_TIMEOUT);
				taskEXIT_CRITICAL();

				delay_us(25);
				ADS_chipRelease(&(connected_chips[k]));

				if(connected_chips[k].current_mux == 2) {
					if(xSemaphoreTake(ADS_main->temp_semaphore, (TickType_t) 2) == pdTRUE) {
						ADS_main->raw_temp = ((MSB << 8) | LSB) & 0xFFFC;

						xSemaphoreGive(ADS_main->temp_semaphore);
					}
					ADS_main->last_temp = HAL_GetTick();
				}
				else {
					if(xSemaphoreTake(connected_chips[k].muxes[connected_chips[k].current_mux]->reading_semaphore, (TickType_t) 2) == pdTRUE) {
						connected_chips[k].muxes[connected_chips[k].current_mux]->current_raw = (MSB << 8) | LSB;
						connected_chips[k].muxes[connected_chips[k].current_mux]->timestamp = getTimestamp();
						xSemaphoreGive(connected_chips[k].muxes[connected_chips[k].current_mux]->reading_semaphore);
					}
				}

				connected_chips[k].current_mux = next_index;
				connected_chips[k].last_tick = HAL_GetTick();
				connected_chips[k].timer_start = HAL_GetTick();
			}
		}
	}
}

HAL_StatusTypeDef ADS_write(ADS_Chip *ADS, uint8_t *tx_buffer, uint8_t num_bytes) {
	ADS_chipSelect(ADS);

	delay_us(50);
	taskENTER_CRITICAL();
	HAL_StatusTypeDef status = HAL_SPI_Transmit(ADS->hspi, tx_buffer, num_bytes, ADS->SPI_TIMEOUT);
	taskEXIT_CRITICAL();

	delay_us(25);
	ADS_chipRelease(ADS);

	return status;
}

HAL_StatusTypeDef ADS_read(ADS_Chip *ADS, uint8_t reg_addr, uint8_t *rx_buffer, uint8_t num_bytes) {
	taskENTER_CRITICAL();

	ADS_chipSelect(ADS);
	HAL_SPI_Transmit(ADS->hspi, &reg_addr, 1, ADS->SPI_TIMEOUT);
	HAL_StatusTypeDef status = HAL_SPI_Receive(ADS->hspi, rx_buffer, num_bytes, ADS->SPI_TIMEOUT);
	ADS_chipRelease(ADS);

	taskEXIT_CRITICAL();

	return status;
}

void ADS_chipSelect(ADS_Chip *ADS) {
	HAL_GPIO_WritePin(ADS->CS_GPIO_Port, ADS->CS_GPIO_Pin, 0);
}

void ADS_chipRelease(ADS_Chip *ADS) {
	HAL_GPIO_WritePin(ADS->CS_GPIO_Port, ADS->CS_GPIO_Pin, 1);
}

int ADS_configure(ADS_Chip *ADS, uint8_t TC_index, int check) {
	uint8_t conf_byte_1 = 0x00;
	uint8_t conf_byte_2 = 0x00;
	uint8_t conf_byte_3 = 0x00;
	uint8_t conf_byte_4 = 0x00;

	if(TC_index == 2) {
		conf_byte_1 = 0x00 | 0x00 | ADS_PGA_ENABLED;
		conf_byte_2 = ADS_INTERNAL_TEMP_RATE | ADS_MODE_NORMAL | ADS_CONV_MODE_SING | ADS_INTERNAL_TEMP_ENABLED | ADS_BURN_OUT_DISABLED;
		conf_byte_3 = ADS_VOLT_REF_INT | ADS_FILTER_DISABLED | ADS_PSW_DISABLED | ADS_IDAC_DISABLED;
		conf_byte_4 = ADS_l1MUX_DISABLED | ADS_l2MUX_DISABLED | ADS_DRDY_MODE_BOTH;
	}
	else {
		conf_byte_1 = ADS->muxes[TC_index]->mux | ADS->muxes[TC_index]->gain | ADS_PGA_ENABLED;
		conf_byte_2 = ADS->muxes[TC_index]->rate | ADS_MODE_NORMAL | ADS_CONV_MODE_SING | ADS_INTERNAL_TEMP_DISABLED | ADS_BURN_OUT_DISABLED;
		conf_byte_3 = ADS_VOLT_REF_INT | ADS_FILTER_DISABLED | ADS_PSW_DISABLED | ADS_IDAC_DISABLED;
		conf_byte_4 = ADS_l1MUX_DISABLED | ADS_l2MUX_DISABLED | ADS_DRDY_MODE_BOTH;
	}

	uint8_t txbuffer[5] = {0x43, conf_byte_1, conf_byte_2, conf_byte_3, conf_byte_4};

	ADS_write(ADS, txbuffer, 5);

	if(check) {
		uint8_t rxbuffer[4] = {0, 0, 0, 0};

		ADS_read(ADS, 0x23, rxbuffer, 4);

		return !(rxbuffer[0] == conf_byte_1 && rxbuffer[1] == conf_byte_2 && rxbuffer[2] == conf_byte_3 && rxbuffer[3] == conf_byte_4);
	}
	else {
		return 0;
	}
}

int ADS_configureChip(ADS_Chip *ADS, int check) {
	uint8_t conf_byte_3 = ADS_VOLT_REF_INT | ADS_FILTER_DISABLED | ADS_PSW_DISABLED | ADS_IDAC_DISABLED;
	uint8_t conf_byte_4 = ADS_l1MUX_DISABLED | ADS_l2MUX_DISABLED | ADS_DRDY_MODE_BOTH;

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

int ADS_readIndividual(ADS_Main_t *ADSMain, uint8_t TC_index, float *reading) {
	if(xSemaphoreTake(ADSMain->TCs[TC_index].reading_semaphore, (TickType_t) 2) == pdTRUE) {
		int16_t raw = ADSMain->TCs[TC_index].current_raw;
		xSemaphoreGive(ADSMain->TCs[TC_index].reading_semaphore);

		if(xSemaphoreTake(ADSMain->temp_semaphore, (TickType_t) 2) == pdTRUE) {
			int16_t raw_ref = ADSMain->raw_temp;
			xSemaphoreGive(ADSMain->temp_semaphore);

			double process_reading = ADS_convertRawToMicrovolts(ADSMain->TCs + TC_index, raw);
			process_reading += ADS_polyTempToMicrovolts(raw_ref * 0.0078125);
			*reading = ADS_polyMicrovoltsToTemp(process_reading);
			//*reading = raw_ref * 0.0078125; //DEBUG

			return 0;
		}
	}
	return 1;
}

int ADS_readIndividualwTimestamp(ADS_Main_t *ADSMain, uint8_t TC_index, ADS_Reading_t *reading) {
	if(xSemaphoreTake(ADSMain->TCs[TC_index].reading_semaphore, (TickType_t) 2) == pdTRUE) {
		int16_t raw = ADSMain->TCs[TC_index].current_raw;
		reading->timestamp = ADSMain->TCs[TC_index].timestamp;
		xSemaphoreGive(ADSMain->TCs[TC_index].reading_semaphore);

		if(xSemaphoreTake(ADSMain->temp_semaphore, (TickType_t) 2) == pdTRUE) {
			int16_t raw_ref = ADSMain->raw_temp;
			xSemaphoreGive(ADSMain->temp_semaphore);

			double process_reading = ADS_convertRawToMicrovolts(ADSMain->TCs + TC_index, raw);
			process_reading += ADS_polyTempToMicrovolts(raw_ref * 0.0078125);
			reading->temp_c = ADS_polyMicrovoltsToTemp(process_reading);
			reading->error = 0;

			return 0;
		}
	}
	reading->error = 1;
	return 1;
}

int ADS_readAll(ADS_Main_t *ADSMain, float *readings) {
	if(xSemaphoreTake(ADSMain->temp_semaphore, (TickType_t) 2) == pdTRUE) {
		int16_t raw_ref = ADSMain->raw_temp;
		xSemaphoreGive(ADSMain->temp_semaphore);

		for(int k = 0; k < ADSMain->TC_count; k++) {
			if(xSemaphoreTake(ADSMain->TCs[k].reading_semaphore, (TickType_t) 2) == pdTRUE) {
				int16_t raw_read = ADSMain->TCs[k].current_raw;
				xSemaphoreGive(ADSMain->TCs[k].reading_semaphore);

				double process_reading = ADS_convertRawToMicrovolts(ADSMain->TCs + k, raw_read);
				process_reading += ADS_polyTempToMicrovolts(raw_ref * 0.0078125);
				readings[k] = ADS_polyMicrovoltsToTemp(process_reading);
			}
			else {
				readings[k] = NAN;
			}
		}
		return 0;
	}

	return 1;
}

int ADS_readAllwTimestamps(ADS_Main_t *ADSMain, ADS_Reading_t *readings) {
	if(xSemaphoreTake(ADSMain->temp_semaphore, (TickType_t) 2) == pdTRUE) {
		int16_t raw_ref = ADSMain->raw_temp;
		xSemaphoreGive(ADSMain->temp_semaphore);

		for(int k = 0; k < ADSMain->TC_count; k++) {
			if(xSemaphoreTake(ADSMain->TCs[k].reading_semaphore, (TickType_t) 2) == pdTRUE) {
				int16_t raw_read = ADSMain->TCs[k].current_raw;
				readings[k].timestamp = ADSMain->TCs[k].timestamp;
				xSemaphoreGive(ADSMain->TCs[k].reading_semaphore);

				double process_reading = ADS_convertRawToMicrovolts(ADSMain->TCs + k, raw_read);
				process_reading += ADS_polyTempToMicrovolts(raw_ref * 0.0078125);
				readings[k].temp_c = ADS_polyMicrovoltsToTemp(process_reading);
				readings[k].error = 0;
			}
			else {
				readings[k].error = 1;
			}
		}
		return 0;
	}

	return 1;
}

int ADS_readInternalTemp(ADS_Main_t *ADSMain, float *temp) {
	if(xSemaphoreTake(ADSMain->temp_semaphore, (TickType_t) 2) == pdTRUE) {
		int16_t raw_ref = ADSMain->raw_temp;
		xSemaphoreGive(ADSMain->temp_semaphore);
		*temp = (float) (raw_ref * 0.0078125);
		return 0;
	}
	return 1;
}

int ADS_switchConf(ADS_Chip *ADS, uint8_t TC_index) {
	uint8_t conf_byte_1 = ADS->muxes[TC_index]->mux | ADS->muxes[TC_index]->gain | ADS_PGA_ENABLED;
	uint8_t conf_byte_2 = ADS->muxes[TC_index]->rate | ADS_MODE_NORMAL | ADS_CONV_MODE_SING | ADS_INTERNAL_TEMP_DISABLED | ADS_BURN_OUT_DISABLED;

	uint8_t txbuffer[3] = {0x41, conf_byte_1, conf_byte_2};

	ADS_write(ADS, txbuffer, 3);
	return 1;
}
