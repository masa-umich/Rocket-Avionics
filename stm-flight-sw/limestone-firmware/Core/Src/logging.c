/*
 * logging.c
 *
 *  Created on: Oct 19, 2025
 *      Author: felix
 */


#include "logging.h"

W25N04KV_Flash flash_h = {0};
SemaphoreHandle_t flash_mutex;
SemaphoreHandle_t flash_clear_mutex;
uint8_t flashreadbuffer[2048 + 512];
uint16_t validflashbytes = 0;
uint8_t flashmsgtype = 0;

SemaphoreHandle_t errormsg_mutex;
uint8_t errormsgtimers[ERROR_MSG_TYPES / 2]; // Use 4 bits to store each timer

SemaphoreHandle_t perierrormsg_mutex;
uint8_t perierrormsgtimers[PERI_ERROR_MSG_TYPES];

struct netconn *errormsgudp = NULL;
SemaphoreHandle_t errorudp_mutex;

QueueHandle_t errorMsglist;

uint32_t lastflashfull = 0;

void init_network_logging(uint8_t reinit, ip4_addr_t ipaddr) {
	if(xSemaphoreTake(errorudp_mutex, portMAX_DELAY) == pdPASS) {
		if(!errormsgudp) {
			errormsgudp = netconn_new(NETCONN_UDP);
			if(errormsgudp) {
				ip_set_option(errormsgudp->pcb.udp, SOF_BROADCAST);
				send_udp_online(&ipaddr);
			}
			else {
				// failed to create netconn
				log_message(reinit ? ERR_UDP_REINIT : ERR_UDP_INIT_NNETCONN, -1);
			}
		}
		else {
			log_message(ERR_UDP_REINIT, -1);
		}
		xSemaphoreGive(errorudp_mutex);
	}
	else {
		// failed to take mutex
		log_message(reinit ? ERR_UDP_REINIT : ERR_UDP_INIT_MUTEX, -1);
	}
}

void deinit_network_logging() {
	if(xSemaphoreTake(errorudp_mutex, portMAX_DELAY) == pdPASS) {
		netconn_close(errormsgudp);
		netconn_delete(errormsgudp);
		errormsgudp = NULL;
		xSemaphoreGive(errorudp_mutex);
	}
}

void logging_setup() {
	flash_mutex = xSemaphoreCreateMutex();
	flash_clear_mutex = xSemaphoreCreateBinary();
	errormsg_mutex = xSemaphoreCreateMutex();
	perierrormsg_mutex = xSemaphoreCreateMutex();
	errorudp_mutex = xSemaphoreCreateMutex();

	xSemaphoreTake(flash_clear_mutex, 0); // Make sure the mutex is taken

	memset(errormsgtimers, 0, ERROR_MSG_TYPES / 2);
	memset(perierrormsgtimers, 0, PERI_ERROR_MSG_TYPES);
	errorMsglist = xQueueCreate(100, sizeof(errormsg_t));
}

uint8_t init_flash_logging(SPI_HandleTypeDef * hspi, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_GPIO_Pin) {
	fc_init_flash(&flash_h, hspi, CS_GPIO_Port, CS_GPIO_Pin);
	if(!fc_ping_flash(&flash_h)) {
		// flash not connected
		log_message(ERR_FLASH_INIT, -1);
		return 1;
	}
	return 0;
}

void log_telemetry() {
	Message telemsg = {0};
	telemsg.type = MSG_TELEMETRY;
	if(!pack_fc_telemetry_msg(&(telemsg.data.telemetry), get_rtc_time(), 5)) {
		uint8_t tempbuffer[11 + (4 * FC_TELEMETRY_CHANNELS) + 5];
		int buflen = serialize_message(&telemsg, tempbuffer, 11 + (4 * FC_TELEMETRY_CHANNELS) + 5);
		if(buflen != -1) {
			if(log_lmp_packet(tempbuffer, buflen) == 2) {
				if(lastflashfull == 0 || HAL_GetTick() - lastflashfull > 5000) {
					send_flash_full();
					lastflashfull = HAL_GetTick();
				}
			}
		}
	}
}

void log_valve_states() {
	if(xSemaphoreTake(Rocket_h.fcValve_access, 5) == pdPASS) {
	  	Valve_State_t vstates[3];
		for(int i = 0;i < 3;i++) {
			vstates[i] = Rocket_h.fcValveStates[i];
		}
	  	xSemaphoreGive(Rocket_h.fcValve_access);
		uint64_t valvetime = get_rtc_time();
		for(int i = 0;i < 3;i++) {
			Message statemsg = {0};
			statemsg.type = MSG_VALVE_STATE;
			statemsg.data.valve_state.timestamp = valvetime;
			statemsg.data.valve_state.valve_id = generate_valve_id(BOARD_FC, i);
			statemsg.data.valve_state.valve_state = vstates[i];
			uint8_t tempbuffer[MAX_VALVE_STATE_MSG_SIZE + 5];
			int buflen = serialize_message(&statemsg, tempbuffer, MAX_VALVE_STATE_MSG_SIZE + 5);
			if(buflen != -1) {
				if(log_lmp_packet(tempbuffer, buflen) == 2) {
					if(lastflashfull == 0 || HAL_GetTick() - lastflashfull > 5000) {
						send_flash_full();
						lastflashfull = HAL_GetTick();
					}
				}
			}
		}
	}
}

void handle_logging() {
	errormsg_t logmsg;
	if(xQueueReceive(errorMsglist, (void *)&logmsg, 5) == pdPASS) {
		uint8_t flashstat = write_raw_to_flash(logmsg.content, logmsg.len);
		if(flashstat == 2) {
			// Flash is full, send UDP message every 5 seconds
			if(lastflashfull == 0 || HAL_GetTick() - lastflashfull > 5000) {
				send_flash_full();
				lastflashfull = HAL_GetTick();
			}
		}
		if(xSemaphoreTake(errorudp_mutex, 5) == pdPASS) {
			if(errormsgudp) {
				struct netbuf *outbuf = netbuf_new();
				if(outbuf) {
					void *pkt_buf = netbuf_alloc(outbuf, logmsg.len - 2);
					if(pkt_buf) {
						memcpy(pkt_buf, &(logmsg.content[1]), logmsg.len - 2);
						netconn_sendto(errormsgudp, outbuf, IP4_ADDR_BROADCAST, ERROR_UDP_PORT);
					}
					netbuf_delete(outbuf);
				}
			}
			xSemaphoreGive(errorudp_mutex);
		}

		free(logmsg.content); // No memory leaks here hehe
	}
}

void prepare_flash_dump() {
	xSemaphoreTake(flash_mutex, portMAX_DELAY);
	fc_finish_flash_write(&flash_h);
	fc_reset_flash_read_pointer(&flash_h);
	validflashbytes = 0;
	flashmsgtype = 0;
}

void finish_flash_dump() {
	xSemaphoreGive(flash_mutex);
}

int dump_flash(uint32_t fd, void *buf, int bytes) {
	uint8_t *flashbuf = (uint8_t *) malloc(2048);
	if(!flashbuf) {
		return -1;
	}
	while(bytes > validflashbytes) {
		if(fc_flash_current_page(&flash_h) >= 4 * W25N01GV_NUM_PAGES) {
			break;
		}
		uint16_t cursor = 0;
		uint8_t empty = 1;
		fc_read_next_2KB_from_flash(&flash_h, flashbuf);
		if(flashbuf[0] != FLASH_MSG_MARK && flashbuf[0] != FLASH_TELEM_MARK) {
			// Load or "discard" partial message, be careful if the entire 2048 bytes are part of the partial message
			for(cursor = 0;cursor < 2048;cursor++) {
				if(flashbuf[cursor] != 0xFF) {
					empty = 0;
				}
				if(flashbuf[cursor] == '\n') {
					break;
				}
			}
			cursor += 1;
			if(cursor > 2047) {
				if(empty) {
					break;
				}
				// Load entire or none, then continue
				if(fd == flashmsgtype) {
					memcpy(&flashreadbuffer[validflashbytes], flashbuf, 2048);
					validflashbytes += 2048;
				}
				continue;
			}
			else {
				if(fd == flashmsgtype) {
					memcpy(&flashreadbuffer[validflashbytes], flashbuf, cursor);
					validflashbytes += cursor;
				}
			}
		}
		int start = -1;
		for(;cursor < 2048;cursor++) {
			if(flashbuf[cursor] == FLASH_MSG_MARK || flashbuf[cursor] == FLASH_TELEM_MARK) {
				flashmsgtype = flashbuf[cursor];
				start = cursor + 1;
			}
			else if(flashbuf[cursor] == '\n') {
				if(fd == flashmsgtype) {
					if(start != -1) {
						memcpy(&flashreadbuffer[validflashbytes], &flashbuf[start], (cursor - start) + 1);
						validflashbytes += (cursor - start) + 1;
					}
				}
				start = -1;
			}
		}
		if(start != -1) {
			if(fd == flashmsgtype) {
				memcpy(&flashreadbuffer[validflashbytes], &flashbuf[start], (2047 - start) + 1);
				validflashbytes += (2047 - start) + 1;
			}
		}
	}
	// Load validflashbytes into buf, keep in mind it could be less than 512 or even 0
	int readbytes = (bytes > validflashbytes) ? validflashbytes : bytes;
	memcpy(buf, flashreadbuffer, readbytes);
	memmove(&flashreadbuffer, &flashreadbuffer[readbytes], validflashbytes - readbytes);
	validflashbytes -= readbytes;
	free(flashbuf);
	return readbytes;
}

// Writes text to flash, msgtext does not have to be null terminated and msglen should not include the null character if it is included
// type is the msg type, use the macros in main.h
// returns 0 on success, 1 if there is not enough space or the flash could not be accessed
uint8_t write_ascii_to_flash(const char *msgtext, size_t msglen, uint8_t type) {
	if(xSemaphoreTake(flash_mutex, 0) == pdPASS) {
		if(fc_get_bytes_remaining(&flash_h) < msglen + 2) {
			xSemaphoreGive(flash_mutex);
			return 1;
		}
		uint8_t *writebuf = (uint8_t *) malloc(msglen + 2);
		writebuf[0] = type;
		memcpy(&writebuf[1], msgtext, msglen);
		writebuf[msglen + 1] = '\n';
		fc_write_to_flash(&flash_h, writebuf, msglen + 2);
		free(writebuf);
		xSemaphoreGive(flash_mutex);
		return 0;
	}
	return 1;
}

// Returns 0 on success, 1 on access error, 2 if the flash is full
uint8_t write_raw_to_flash(uint8_t *writebuf, size_t msglen) {
	if(xSemaphoreTake(flash_mutex, 0) == pdPASS) {
		if(fc_get_bytes_remaining(&flash_h) < msglen) {
			xSemaphoreGive(flash_mutex);
			return 2;
		}
		fc_write_to_flash(&flash_h, writebuf, msglen);
		xSemaphoreGive(flash_mutex);
		return 0;
	}
	return 1;
}

/*
 * Log message to flash. This is the function to use throughout the rest of the code.
 * msgtext - Message text. This should include an error code if the message is an error
 * msgtype - Type of message, used for error message throttling. Pass -1 if this is a status message or if you don't want it to be throttled, otherwise pass the "category" of the error message
 * Returns 0 on success, 1 if a message of this type was sent too recently, 2 on general error, and 3 if there isn't enough memory available
 */
uint8_t log_message(const char *msgtext, int msgtype) {
	if(msgtype != -1) {
		if(msgtype >= ERROR_MSG_TYPES) {
			return 2;
		}
		if(xSemaphoreTake(errormsg_mutex, 2) == pdPASS) {
			uint8_t val = (msgtype % 2) == 0 ? errormsgtimers[msgtype / 2] & 0x0F : errormsgtimers[msgtype / 2] >> 4;
			if(val < ERROR_THROTTLE_MAX) {
				val++;
				if(msgtype % 2) {
					errormsgtimers[msgtype / 2] = (errormsgtimers[msgtype / 2] & 0x0F) | (val << 4);
				}
				else {
					errormsgtimers[msgtype / 2] = (errormsgtimers[msgtype / 2] & 0xF0) | val;
				}
			}
			else {
				xSemaphoreGive(errormsg_mutex);
				return 1;
			}
			/*if(val) {
				xSemaphoreGive(errormsg_mutex);
				return 1;
			}
			if(msgtype % 2) {
				errormsgtimers[msgtype / 2] = (errormsgtimers[msgtype / 2] & 0x0F) | (15 << 4);
			}
			else {
				errormsgtimers[msgtype / 2] = (errormsgtimers[msgtype / 2] & 0xF0) | 15;
			}*/
			xSemaphoreGive(errormsg_mutex);
		}
		else {
			return 2;
		}
	}
	// Flash entry type + timestamp + space + error code board number + message text + newline
	size_t msglen = 1 + 24 + 1 + 1 + strlen(msgtext) + 1;
	uint8_t *rawmsgbuf = (uint8_t *) malloc(msglen);
	if(rawmsgbuf) {
		rawmsgbuf[0] = FLASH_MSG_MARK;
		get_iso_time((char *) &rawmsgbuf[1]);
		rawmsgbuf[25] = ' ';
		rawmsgbuf[26] = '1';
		memcpy(&rawmsgbuf[27], msgtext, strlen(msgtext));
		rawmsgbuf[msglen - 1] = '\n';
		errormsg_t fullmsg;
		fullmsg.content = rawmsgbuf;
		fullmsg.len = msglen;
		if(xQueueSend(errorMsglist, (void *)&fullmsg, 1) != pdPASS) {
			// No space for more messages
			free(rawmsgbuf);
			return 3;
		}
		return 0;
	}
	return 3;
}

// Same as log_message but intended for use with peripheral device errors
// Only difference is the message type timers last a lot longer ~ 5 seconds
// This is intended to reduce message spam in the case that we intentionally run boards with disconnected peripherals
uint8_t log_peri_message(const char *msgtext, int msgtype) {
	if(msgtype != -1) {
		if(msgtype >= PERI_ERROR_MSG_TYPES) {
			return 2;
		}
		if(xSemaphoreTake(perierrormsg_mutex, 2) == pdPASS) {
			if(perierrormsgtimers[msgtype]) {
				xSemaphoreGive(perierrormsg_mutex);
				return 1;
			}
			perierrormsgtimers[msgtype] = 20; // 20 * 250 = ~5000ms
			xSemaphoreGive(perierrormsg_mutex);
		}
		else {
			return 2;
		}
	}
	// Flash entry type + timestamp + space + error code board number + message text + newline
	size_t msglen = 1 + 24 + 1 + 1 + strlen(msgtext) + 1;
	uint8_t *rawmsgbuf = (uint8_t *) malloc(msglen);
	if(rawmsgbuf) {
		rawmsgbuf[0] = FLASH_MSG_MARK;
		get_iso_time((char *) &rawmsgbuf[1]);
		rawmsgbuf[25] = ' ';
		rawmsgbuf[26] = '1';
		memcpy(&rawmsgbuf[27], msgtext, strlen(msgtext));
		rawmsgbuf[msglen - 1] = '\n';
		errormsg_t fullmsg;
		fullmsg.content = rawmsgbuf;
		fullmsg.len = msglen;
		if(xQueueSend(errorMsglist, (void *)&fullmsg, 1) != pdPASS) {
			// No space for more messages
			free(rawmsgbuf);
			return 3;
		}
		return 0;
	}
	return 3;
}

void send_flash_full() {
	if(xSemaphoreTake(errorudp_mutex, 5) == pdPASS) {
		if(errormsgudp) {
			struct netbuf *outbuf = netbuf_new();
			if(outbuf) {
				char *pkt_buf = (char *) netbuf_alloc(outbuf, 24 + 1 + 1 + sizeof(ERR_FLASH_FULL) - 1);
				if(pkt_buf) {
					get_iso_time(pkt_buf);
					pkt_buf[24] = ' ';
					pkt_buf[25] = '1';
					memcpy(&pkt_buf[26], ERR_FLASH_FULL, sizeof(ERR_FLASH_FULL) - 1);
					netconn_sendto(errormsgudp, outbuf, IP4_ADDR_BROADCAST, ERROR_UDP_PORT);
				}
				netbuf_delete(outbuf);
			}
		}
		xSemaphoreGive(errorudp_mutex);
	}
}

// 0 success, 1 semaphore timeout, 2 flash full, 3 memory error
int log_lmp_packet(uint8_t *buf, size_t buflen) {
	size_t outlen;
	uint8_t *encoded = base64_encode(buf, buflen, &outlen, 1);
	if(encoded) {
		encoded[0] = FLASH_TELEM_MARK;
		encoded[outlen] = '\n';
		uint8_t stat = write_raw_to_flash(encoded, outlen + 1);
		free(encoded);
		return stat;
	}
	return 3;
}

void send_udp_online(ip4_addr_t * ip) {
	size_t msglen = 24 + 1 + 1 + sizeof(STAT_NETWORK_LOG_ONLINE) + 15;
	struct netbuf *outbuf = netbuf_new();
	if(outbuf) {
		uint8_t *pkt_buf = (uint8_t *) netbuf_alloc(outbuf, msglen);
		if(pkt_buf) {
			get_iso_time((char *) pkt_buf);
			pkt_buf[24] = ' ';
			pkt_buf[25] = '1';
			snprintf((char *) &pkt_buf[26], sizeof(STAT_NETWORK_LOG_ONLINE) + 15, STAT_NETWORK_LOG_ONLINE "%u.%u.%u.%u", ip4_addr1(ip), ip4_addr2(ip), ip4_addr3(ip), ip4_addr4(ip));
			netconn_sendto(errormsgudp, outbuf, IP4_ADDR_BROADCAST, ERROR_UDP_PORT);
		}
		netbuf_delete(outbuf);
	}
}

void log_flash_storage() {
	if(xSemaphoreTake(flash_mutex, 1) == pdPASS) {
		uint32_t used = 536870912UL - fc_get_bytes_remaining(&flash_h);
		xSemaphoreGive(flash_mutex);

		uint8_t percent = (used / 536870912.0f) * 100;
		if(used < 1024) {
	    	char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 22];
	    	snprintf(logmsg, sizeof(logmsg), STAT_AVAILABLE_FLASH "%" PRIu32 "B/512MB %u%%", used, percent);
	    	log_message(logmsg, -1);
		}
		else if(used < 1048576UL) {
	    	char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 36];
	    	snprintf(logmsg, sizeof(logmsg), STAT_AVAILABLE_FLASH "%" PRIu32 "B: %" PRIu32 "KB/512MB %u%%", used, used >> 10, percent);
	    	log_message(logmsg, -1);
		}
		else {
	    	char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 31];
	    	snprintf(logmsg, sizeof(logmsg), STAT_AVAILABLE_FLASH "%" PRIu32 "B: %" PRIu16 "MB/512MB %u%%", used, (uint16_t) (used >> 20), percent);
	    	log_message(logmsg, -1);
		}
	}
}

void clear_flash() {
	xSemaphoreGive(flash_clear_mutex); // signal to task to clear flash. This needs to be async since it takes almost 5 seconds to clear the flash
}

// Note that this blocks almost indefinitely
void handle_flash_clearing() {
    if(xSemaphoreTake(flash_clear_mutex, portMAX_DELAY) == pdPASS) {
    	if(xSemaphoreTake(flash_mutex, 100) == pdPASS) {
    		fc_finish_flash_write(&flash_h); // Flush write buffer
        	fc_erase_flash(&flash_h); // Clear
    		xSemaphoreGive(flash_mutex);
    		log_message(STAT_CLEAR_FLASH, -1);
    	}
    }
}

// 1 if online, 0 otherwise
uint8_t is_net_logging_up() {
	uint8_t status = 0;
	if(xSemaphoreTake(errorudp_mutex, portMAX_DELAY) == pdPASS) {
		if(errormsgudp) {
			status = 1;
		}
		xSemaphoreGive(errorudp_mutex);
	}
	return status;
}

void FlashClearTask(void *argument) {
    for(;;) {
    	handle_flash_clearing();
    }
}

void flush_flash_log() {
	if(xSemaphoreTake(flash_mutex, 500) == pdPASS) {
		fc_finish_flash_write(&flash_h);
		xSemaphoreGive(flash_mutex);
	}
}

void refresh_log_timers() {
	if(xSemaphoreTake(errormsg_mutex, 5) == pdPASS) {
		for(int i = 0;i < ERROR_MSG_TYPES;i++) {
			uint8_t val = (i % 2) == 0 ? errormsgtimers[i / 2] & 0x0F : errormsgtimers[i / 2] >> 4;
			if(val) {
				val--;
				if(i % 2) {
					errormsgtimers[i / 2] = (errormsgtimers[i / 2] & 0x0F) | (val << 4);
				}
				else {
					errormsgtimers[i / 2] = (errormsgtimers[i / 2] & 0xF0) | val;
				}
			}
		}
		xSemaphoreGive(errormsg_mutex);
	}

	if(xSemaphoreTake(perierrormsg_mutex, 5) == pdPASS) {
		for(int i = 0;i < PERI_ERROR_MSG_TYPES;i++) {
			if(perierrormsgtimers[i]) {
				perierrormsgtimers[i]--;
			}
		}
		xSemaphoreGive(perierrormsg_mutex);
	}
}
