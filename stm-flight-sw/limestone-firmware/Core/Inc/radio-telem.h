/*
 * radio-telem.h
 *
 *  Created on: Apr 12, 2026
 *      Author: felix
 */

#ifndef INC_RADIO_TELEM_H_
#define INC_RADIO_TELEM_H_

#include "main.h"
#include "cmsis_os.h"
#include "sx1280.h"
#include "system-state.h"

#define LORA_SYNC_WORD 0x12

typedef struct {
	uint16_t fuck;
} radio_telem_t;

#define RADIO_TELEM_SIZE

uint8_t setup_radio(uint8_t broadcast, SX1280_Hal_t * config);

void BroadcastRadio(void *argument);

void init_radio(uint8_t enable);

void set_broadcast_state(uint8_t enable);

#endif /* INC_RADIO_TELEM_H_ */
