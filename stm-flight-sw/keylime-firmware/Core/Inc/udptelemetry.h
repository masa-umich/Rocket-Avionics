/*
 * udptelemetry.h
 *
 *  Created on: Dec 6, 2025
 *      Author: felix
 */

#ifndef INC_UDPTELEMETRY_H_
#define INC_UDPTELEMETRY_H_

#include "main.h"
#include "client.h"
#include "logging.h"
#include "queue.h"
#include "api.h"
#include "lwip/udp.h"
#include "ip4_addr.h"
#include "eeprom-config.h"

void switch_telem_logging(uint8_t enable);

uint8_t telemetry_setup(uint8_t enabled);

void init_udp_telem();

void deinit_udp_telem();

int broadcast_telem_msg(Message *msg, TickType_t wait);

void TelemetrySend(void *argument);

void log_udp_telemetry(uint8_t *message, uint16_t msg_len);

void LogListener(void *argument);

void log_udp_log(uint8_t *message, uint16_t msg_len);

#endif /* INC_UDPTELEMETRY_H_ */
