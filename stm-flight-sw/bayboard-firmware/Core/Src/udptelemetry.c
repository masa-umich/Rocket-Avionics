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

void telemetry_setup() {
	telemudp_mutex = xSemaphoreCreateMutex();
	telemetrymsgs = xQueueCreate(100, sizeof(RawMessage));
}

void init_udp_telem() {
	if(xSemaphoreTake(telemudp_mutex, portMAX_DELAY) == pdPASS) {
		if(!telemudp_h) {
			telemudp_h = netconn_new(NETCONN_UDP);
			if(telemudp_h) {
				ip_set_option(telemudp_h->pcb.udp, SOF_BROADCAST);
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

int broadcast_telem_msg(Message *msg, TickType_t wait, size_t buffersize) {
	if(buffersize == 0) {
		buffersize = MAX_MSG_LEN;
	}
	if(!telemudp_h) {
		return -1;
	}
	uint8_t tempbuffer[buffersize];
	int buflen = serialize_message(msg, tempbuffer, buffersize);
	if(buflen == -1) {
		return -3; // Serialization error
	}
    uint8_t *buffer = malloc(buflen);
    if(buffer) {
        memcpy(buffer, tempbuffer, buflen);
    	RawMessage rawmsg = {0};
    	rawmsg.bufferptr = buffer;
    	rawmsg.packet_len = buflen;

    	if(xQueueSend(telemetrymsgs, (void *)&rawmsg, wait) != pdPASS) {
    		free(buffer);
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
			free(msg.bufferptr);
		}
		osDelay(1);
	}
}
