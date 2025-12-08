/*
 * udptelemetry.h
 *
 *  Created on: Dec 6, 2025
 *      Author: felix
 */

#ifndef INC_UDPTELEMETRY_H_
#define INC_UDPTELEMETRY_H_

#include "main.h"
#include "server.h"
#include "logging.h"
#include "queue.h"
#include "api.h"
#include "lwip/udp.h"
#include "ip4_addr.h"

void telemetry_setup();

void init_udp_telem();

void deinit_udp_telem();

int broadcast_telem_msg(Message *msg, TickType_t wait, size_t buffersize);

void TelemetryUDPProcess(void *argument);

void unpack_udp_telemetry(uint8_t *message, uint16_t msg_len);

#endif /* INC_UDPTELEMETRY_H_ */
