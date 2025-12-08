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
	telemetrymsgs = xQueueCreate(100, sizeof(Raw_message));
}

void init_udp_telem() {
	if(xSemaphoreTake(telemudp_mutex, portMAX_DELAY) == pdPASS) {
		if(!telemudp_h) {
			telemudp_h = netconn_new(NETCONN_UDP);
			if(telemudp_h) {
				ip_set_option(telemudp_h->pcb.udp, SOF_BROADCAST);
				netconn_set_recvtimeout(telemudp_h, 10);
				netconn_bind(telemudp_h, IP4_ADDR_ANY, TELEM_UDP_PORT);
			}
			else {
				// failed to create netconn
				log_message(ERR_UDP_REINIT, -1); // TODO
			}
		}
		else {
			log_message(ERR_UDP_REINIT, -1); // TODO
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
        Raw_message rawmsg = {0};
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

void TelemetryUDPProcess(void *argument) {
	for(;;) {
		// Send if packet available
		Raw_message msg = {0};
		if(xQueueReceive(telemetrymsgs, (void *)&msg, 0) == pdPASS) {
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

		// Receive
		struct netbuf *buf = NULL;
		if(xSemaphoreTake(telemudp_mutex, 5) == pdPASS) {
			err_t recv_err = netconn_recv(telemudp_h, &buf);
			if(recv_err == ERR_OK) {
				if(buf) {
					uint8_t *msgbuf = NULL;
					uint16_t msg_len = 0;
					if(netbuf_data(buf, (void**) &msgbuf, &msg_len) == ERR_OK) {
						unpack_udp_telemetry(msgbuf, msg_len);
					}
					netbuf_delete(buf);
				}
			}
			xSemaphoreGive(telemudp_mutex);
		}
	}
}

void unpack_udp_telemetry(uint8_t *message, uint16_t msg_len) {
	Message parsedmsg = {0};
	if(deserialize_message(message, msg_len, &parsedmsg) > 0) {
		switch(parsedmsg.type) {
		    case MSG_TELEMETRY: {
		        // Save
		    	if(parsedmsg.data.telemetry.board_id == BOARD_FR) {
		    		if(unpack_fr_telemetry(&(parsedmsg.data.telemetry), 5)) {
		    			// Failed to save data
		    			log_message(ERR_SAVE_INCOMING_TELEM, FC_ERR_TYPE_INCOMING_TELEM);
		    		}
		    	}
		    	else {
			    	if(unpack_bb_telemetry(&(parsedmsg.data.telemetry), 5)) {
			    		// Failed to save data
			    		log_message(ERR_SAVE_INCOMING_TELEM, FC_ERR_TYPE_INCOMING_TELEM);
			    	}
		    	}
		        break;
		    }
		    default: {
		        break;
		    }
		}
	}
	else {
		// Unknown message type
		log_message(ERR_UNKNOWN_LMP_PACKET, FC_ERR_TYPE_UNKNOWN_LMP);
	}
}
