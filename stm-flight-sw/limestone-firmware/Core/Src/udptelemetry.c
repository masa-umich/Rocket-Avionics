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
	telemetrymsgs = xQueueCreate(50, sizeof(Raw_message));

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
				//netconn_set_recvtimeout(telemudp_h, 10);
				netconn_set_nonblocking(telemudp_h, 1);
				netconn_bind(telemudp_h, IP4_ADDR_ANY, TELEM_UDP_PORT);
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
        Raw_message rawmsg = {0};
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
			osMemoryPoolFree(udpPool, msg.bufferptr);
		}

		// Receive
		struct netbuf *buf = NULL;
		if(xSemaphoreTake(telemudp_mutex, 5) == pdPASS) {
			if(telemudp_h) {
				err_t recv_err = netconn_recv(telemudp_h, &buf);
				if(recv_err == ERR_OK) {
					if(buf) {
						uint8_t msgbuf[256];
						uint16_t msg_len = netbuf_len(buf);
						if(msg_len > 256 || msg_len == 0) {
							log_message(FC_ERR_UDP_TELEM_RECV_SIZE_ERR, FC_ERR_TYPE_UDP_TELEM);
						}
						else {
							uint16_t ret = netbuf_copy(buf, msgbuf, msg_len);
							if(ret != 0 && ret == msg_len) {
								unpack_udp_telemetry(msgbuf, msg_len);
							}
						}
						netbuf_delete(buf);
					}
				}
			}
			else {
				// If not connected, delay to give cpu to other tasks
				xSemaphoreGive(telemudp_mutex);
				osDelay(500);
				continue;
			}
			xSemaphoreGive(telemudp_mutex);
		}
		osDelay(1);
	}
}

void unpack_udp_telemetry(uint8_t *message, uint16_t msg_len) {
	if(message[0] + 1 != msg_len) {
		log_message(FC_ERR_UDP_TELEM_RECV_SIZE_MISMATCH, FC_ERR_TYPE_UDP_TELEM);
		return;
	}
	Message parsedmsg = {0};
	if(deserialize_message(message, msg_len, &parsedmsg) > 0) {
		switch(parsedmsg.type) {
		    case MSG_TELEMETRY: {
		        // Save
		    	if(parsedmsg.data.telemetry.board_id == BOARD_FR) {
		    		if(unpack_fr_telemetry(&(parsedmsg.data.telemetry), 2)) {
		    			// Failed to save data
		    			log_message(ERR_SAVE_INCOMING_TELEM, FC_ERR_TYPE_INCOMING_TELEM);
		    		}
		    	}
		    	else {
			    	if(unpack_bb_telemetry(&(parsedmsg.data.telemetry), 2)) {
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
