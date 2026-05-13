/*
 * radio-telem.c
 *
 *  Created on: Apr 12, 2026
 *      Author: felix
 */

#include "radio-telem.h"

uint8_t global_broadcast_enabled = 0;

uint8_t broadcast_enable = 0;
SemaphoreHandle_t broadcast_mutex;

void init_radio(uint8_t enable) {
	broadcast_enable = enable;
	broadcast_mutex = xSemaphoreCreateMutex();
}

void set_broadcast_state(uint8_t enable) {
	if(xSemaphoreTake(broadcast_mutex, 10) == pdPASS) {
		broadcast_enable = enable;
		xSemaphoreGive(broadcast_mutex);
	}
}

uint8_t setup_radio(uint8_t broadcast, SX1280_Hal_t *config) {
	global_broadcast_enabled = broadcast;

	SX1280_Status_t radio_status = SX1280_Init(config);

	if(radio_status != SX1280_OK) {
		global_broadcast_enabled = 0;
		return 0;
	}

	SX1280_SetLoRaSyncWord(LORA_SYNC_WORD);
	return 1;
}

void BroadcastRadio(void *argument) {
	for(;;) {
		xSemaphoreTake(broadcast_mutex, portMAX_DELAY);
		uint8_t enabled = broadcast_enable;
		xSemaphoreGive(broadcast_mutex);

		if(global_broadcast_enabled && enabled) {
			uint8_t tx_buffer[RADIO_PACKET_LENGTH];
			if(serialize_radio_telem(tx_buffer, RADIO_PACKET_LENGTH, 5)) {
		        SX1280_SetPacketParams(0x0C, LORA_IMPLICIT_HEADER, RADIO_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_STANDARD);
		        SX1280_WriteBuffer(tx_buffer, RADIO_PACKET_LENGTH);
		        SX1280_SetTx(5000); // 5 seconds

		        //uint32_t start = HAL_GetTick();

		        uint16_t irq;

		        do {
		        	osDelay(100);
		            SX1280_GetIrqStatus(&irq);
		        } while(!(irq & (SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT)));

		        //uint32_t delt = HAL_GetTick() - start;

		        SX1280_ClearIrqStatus(0xFFFF);
			}
		}
		osDelay(4000);
	}
}
