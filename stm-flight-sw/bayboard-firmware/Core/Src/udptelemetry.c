/*
 * udptelemetry.c
 *
 *  Created on: Dec 6, 2025
 *      Author: felix
 */

#include "udptelemetry.h"

struct netconn *telemudp_h = NULL;
SemaphoreHandle_t telemudp_mutex;

QueueHandle_t telemetrymsgs = NULL;

osMemoryPoolId_t udpPool;

uint8_t telemetry_setup() {
	telemudp_mutex = xSemaphoreCreateMutex();
	telemetrymsgs = xQueueCreate(50, sizeof(RawMessage));
    udpPool = osMemoryPoolNew(50, MAX_MSG_LEN, NULL);

    if(!udpPool) {
    	return 1;
    }
    return 0;
}

void init_udp_telem() {
	if(xSemaphoreTake(telemudp_mutex, portMAX_DELAY) == pdPASS) {
		if(!telemudp_h) {
			telemudp_h = netconn_new(NETCONN_UDP);
			if(telemudp_h) {
				ip_set_option(telemudp_h->pcb.udp, SOF_BROADCAST);
				netconn_set_nonblocking(telemudp_h, 1);
			}
			else {
				// failed to create netconn
				log_message(ERR_TELEM_UDP_INIT, -1);
			}
		}
		else {
			log_message(ERR_TELEM_UDP_INIT, -1);
		}
		xSemaphoreGive(telemudp_mutex);
	}
}

void deinit_udp_telem() {
	if(xSemaphoreTake(telemudp_mutex, portMAX_DELAY) == pdPASS) {
		netconn_close(telemudp_h);
		netconn_delete(telemudp_h);
		telemudp_h = NULL;
		xSemaphoreGive(telemudp_mutex);
	}
}

int broadcast_telem_msg(Message *msg, TickType_t wait) {
	if(!telemudp_h) {
		return -1;
	}
	uint8_t tempbuffer[MAX_MSG_LEN];
	int buflen = serialize_message(msg, tempbuffer, MAX_MSG_LEN);
	if(buflen == -1) {
		return -3; // Serialization error
	}
    uint8_t *buffer = (uint8_t *) osMemoryPoolAlloc(udpPool, 0);
    if(buffer) {
        memcpy(buffer, tempbuffer, buflen);
    	RawMessage rawmsg = {0};
    	rawmsg.bufferptr = buffer;
    	rawmsg.packet_len = buflen;

    	if(xQueueSend(telemetrymsgs, (void *)&rawmsg, wait) != pdPASS) {
    		osMemoryPoolFree(udpPool, buffer);
    		return -2;
    	}
    	return 0;
    }
    return -2;
}

void TelemetrySend(void *argument) {
	for(;;) {
		RawMessage msg = {0};
		if(xQueueReceive(telemetrymsgs, (void *)&msg, portMAX_DELAY) == pdPASS) {
			if(xSemaphoreTake(telemudp_mutex, 5) == pdPASS) {
				if(telemudp_h) {
					struct netbuf *outbuf = netbuf_new();
					if(outbuf) {
						void *pkt_buf = netbuf_alloc(outbuf, msg.packet_len);
						if(pkt_buf) {
							memcpy(pkt_buf, msg.bufferptr, msg.packet_len);
							netconn_sendto(telemudp_h, outbuf, IP4_ADDR_BROADCAST, TELEM_UDP_PORT);
						}
						netbuf_delete(outbuf);
					}
				}
				xSemaphoreGive(telemudp_mutex);
			}
			osMemoryPoolFree(udpPool, msg.bufferptr);
		}
		osDelay(1);
	}
}
