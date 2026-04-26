/*
 * logging.c
 *
 *  Created on: Oct 31, 2025
 *      Author: felix
 */

#include "logging.h"
#include "utils.h"

W25N04KV_Flash flash_chips[4];
SemaphoreHandle_t flash_mutex;
SemaphoreHandle_t flash_clear_mutex;
SemaphoreHandle_t flash_spi_mutex;
uint8_t flashreadbuffer[2048 + 512];
uint16_t validflashbytes = 0;
uint8_t flashmsgtype = 0;

SemaphoreHandle_t errormsg_mutex;
uint8_t errormsgtimers[ERROR_MSG_TYPES / 2]; // Use 4 bits to store each timer

SemaphoreHandle_t perierrormsg_mutex;
uint8_t perierrormsgtimers[PERI_ERROR_MSG_TYPES];

struct netconn *errormsgudp = NULL;
SemaphoreHandle_t errorudp_mutex;

QueueHandle_t errorMsgList = NULL;

uint32_t lastflashfull = 0;

osMemoryPoolId_t logPool;

BaseType_t LOCK_FLASH(TickType_t timeout) {
	return xSemaphoreTake(flash_spi_mutex, timeout);
}

void UNLOCK_FLASH() {
	xSemaphoreGive(flash_spi_mutex);
}

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

uint8_t logging_setup() {
	flash_mutex = xSemaphoreCreateBinary();
	xSemaphoreGive(flash_mutex);

	flash_clear_mutex = xSemaphoreCreateBinary();
	flash_spi_mutex = xSemaphoreCreateBinary();
	xSemaphoreGive(flash_spi_mutex);

	errormsg_mutex = xSemaphoreCreateMutex();
	perierrormsg_mutex = xSemaphoreCreateMutex();
	errorudp_mutex = xSemaphoreCreateMutex();

	xSemaphoreTake(flash_clear_mutex, 0); // Make sure the mutex is taken

	memset(errormsgtimers, 0, ERROR_MSG_TYPES / 2);
	memset(perierrormsgtimers, 0, PERI_ERROR_MSG_TYPES);

	for(uint8_t i = 0;i < 4;i++) {
		memset(&(flash_chips[i]), 0, sizeof(W25N04KV_Flash));
	}

	errorMsgList = xQueueCreate(25, sizeof(errormsg_t));

	logPool = osMemoryPoolNew(25, MAX_LOG_LEN, NULL);
	if(!logPool) {
		return 1;
	}
	return 0;
}

uint8_t init_flash_logging(SPI_HandleTypeDef * hspi, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint8_t flash_index) {
	init_flash(&(flash_chips[flash_index]), hspi, CS_GPIO_Port, CS_GPIO_Pin);
	if(!ping_flash(&(flash_chips[flash_index]))) {
		// flash not connected
		log_message(ERR_FLASH_INIT, -1);
		return 1;
	}
	return 0;
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
		if(xSemaphoreTake(errormsg_mutex, 1) == pdPASS) {
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
	if(msglen > MAX_LOG_LEN) {
		return 3;
	}
	uint8_t *rawmsgbuf = (uint8_t *) osMemoryPoolAlloc(logPool, 0);
	if(rawmsgbuf) {
		rawmsgbuf[0] = FLASH_MSG_MARK;
		get_iso_time((char *) &rawmsgbuf[1], msglen - 1);
		rawmsgbuf[25] = ' ';
		rawmsgbuf[26] = '4';
		memcpy(&rawmsgbuf[27], msgtext, strlen(msgtext));
		rawmsgbuf[msglen - 1] = '\n';
		errormsg_t fullmsg;
		fullmsg.content = rawmsgbuf;
		fullmsg.len = msglen;
		if(xQueueSend(errorMsgList, (void *)&fullmsg, 0) != pdPASS) {
			// No space for more messages
			osMemoryPoolFree(logPool, rawmsgbuf);
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
	if(msglen > MAX_LOG_LEN) {
		return 3;
	}
	uint8_t *rawmsgbuf = (uint8_t *) osMemoryPoolAlloc(logPool, 0);
	if(rawmsgbuf) {
		rawmsgbuf[0] = FLASH_MSG_MARK;
		get_iso_time((char *) &rawmsgbuf[1], msglen - 1);
		rawmsgbuf[25] = ' ';
		rawmsgbuf[26] = '4';
		memcpy(&rawmsgbuf[27], msgtext, strlen(msgtext));
		rawmsgbuf[msglen - 1] = '\n';
		errormsg_t fullmsg;
		fullmsg.content = rawmsgbuf;
		fullmsg.len = msglen;
		if(xQueueSend(errorMsgList, (void *)&fullmsg, 1) != pdPASS) {
			// No space for more messages
			osMemoryPoolFree(logPool, rawmsgbuf);
			return 3;
		}
		return 0;
	}
	return 3;
}

void handle_logging() {
	errormsg_t logmsg;
	if(xQueueReceive(errorMsgList, (void *)&logmsg, 0) == pdPASS) {
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

		osMemoryPoolFree(logPool, logmsg.content); // No memory leaks here hehe
	}
}

void prepare_flash_dump() {
	xSemaphoreTake(flash_mutex, portMAX_DELAY);
	for(int i = 0;i < 4;i++) {
		finish_flash_write(&(flash_chips[i]));
		reset_flash_read_pointer(&(flash_chips[i]));
	}
	validflashbytes = 0;
	flashmsgtype = 0;
	LOCK_FLASH(portMAX_DELAY);
}

void finish_flash_dump() {
	UNLOCK_FLASH();
	xSemaphoreGive(flash_mutex);
}

int dump_flash(uint32_t fd, void *buf, int bytes) {
	uint8_t chip_index = fd - FLASH_BOTH_MARK;
	uint8_t flashbuf[2048];
	while(bytes > validflashbytes) {
		if(next_read_page(&(flash_chips[chip_index])) >= W25N04KV_NUM_PAGES) {
			break;
		}
		read_next_2KB_from_flash(&(flash_chips[chip_index]), flashbuf);
		uint8_t empty = 1;
		for(uint16_t i = 0;i < 2048;i++) {
			if(flashbuf[i] != 0xFF) {
				empty = 0;
				break;
			}
		}
		if(!empty) {
			memcpy(&flashreadbuffer[validflashbytes], flashbuf, 2048);
			validflashbytes += 2048;
		}
		else {
			break;
		}
	}
	// Load validflashbytes into buf, keep in mind it could be less than 512 or even 0
	int readbytes = (bytes > validflashbytes) ? validflashbytes : bytes;
	memcpy(buf, flashreadbuffer, readbytes);
	memmove(&flashreadbuffer, &flashreadbuffer[readbytes], validflashbytes - readbytes);
	validflashbytes -= readbytes;
	return readbytes;
}

// Returns 0 on success, 1 on access error, 2 if the flash is full
uint8_t write_raw_to_flash(uint8_t *writebuf, size_t msglen) {
	if(xSemaphoreTake(flash_mutex, 0) == pdPASS) {
		uint8_t res = 0;
		for(uint8_t i = 0;i < 4;i++) {
			if(get_bytes_remaining(&(flash_chips[i])) < msglen) {
				res = 1;
				continue;
			}
			write_to_flash(&(flash_chips[i]), writebuf, msglen);
		}
		xSemaphoreGive(flash_mutex);
		return res ? 2 : 0;
	}
	return 1;
}

void log_lmp_packet(uint8_t *buf, size_t buflen) {
	uint8_t outbuf[MAX_TELEMETRY_B64_SIZE];
	size_t outlen = base64_encode(buf, buflen, outbuf, MAX_TELEMETRY_B64_SIZE, 1);
	if(outlen > 0) {
		outbuf[0] = FLASH_TELEM_MARK;
		outbuf[outlen] = '\n';
		uint8_t stat = write_raw_to_flash(outbuf, outlen + 1);
		if(stat == 2) {
			if(lastflashfull == 0 || HAL_GetTick() - lastflashfull > 5000) {
				send_flash_full();
				lastflashfull = HAL_GetTick();
			}
		}
	}
}

void send_flash_full() {
	if(xSemaphoreTake(errorudp_mutex, 5) == pdPASS) {
		if(errormsgudp) {
			struct netbuf *outbuf = netbuf_new();
			if(outbuf) {
				char *pkt_buf = (char *) netbuf_alloc(outbuf, 24 + 1 + 1 + sizeof(ERR_FLASH_FULL) - 1);
				if(pkt_buf) {
					get_iso_time(pkt_buf, 24 + 1 + 1 + sizeof(ERR_FLASH_FULL) - 1);
					pkt_buf[24] = ' ';
					pkt_buf[25] = '4';
					memcpy(&pkt_buf[26], ERR_FLASH_FULL, sizeof(ERR_FLASH_FULL) - 1);
					netconn_sendto(errormsgudp, outbuf, IP4_ADDR_BROADCAST, ERROR_UDP_PORT);
				}
				netbuf_delete(outbuf);
			}
		}
		xSemaphoreGive(errorudp_mutex);
	}
}

void send_udp_online(ip4_addr_t * ip) {
	size_t msglen = 24 + 1 + 1 + sizeof(STAT_NETWORK_LOG_ONLINE) + 15;
	struct netbuf *outbuf = netbuf_new();
	if(outbuf) {
		uint8_t *pkt_buf = (uint8_t *) netbuf_alloc(outbuf, msglen);
		if(pkt_buf) {
			get_iso_time((char *) pkt_buf, msglen);
			pkt_buf[24] = ' ';
			pkt_buf[25] = '4';
			snprintf((char *) &pkt_buf[26], sizeof(STAT_NETWORK_LOG_ONLINE) + 15, STAT_NETWORK_LOG_ONLINE "%u.%u.%u.%u", ip4_addr1(ip), ip4_addr2(ip), ip4_addr3(ip), ip4_addr4(ip));
			netconn_sendto(errormsgudp, outbuf, IP4_ADDR_BROADCAST, ERROR_UDP_PORT);
		}
		netbuf_delete(outbuf);
	}
}

void log_flash_storage(char *logstring, int numbytes) {
	if(xSemaphoreTake(flash_mutex, 1) == pdPASS) {
		//uint32_t used = 536870912UL - fc_get_bytes_remaining(&flash_h);
		uint32_t available[4] = {get_bytes_remaining(&(flash_chips[0])), get_bytes_remaining(&(flash_chips[1])), get_bytes_remaining(&(flash_chips[2])), get_bytes_remaining(&(flash_chips[3]))};
		xSemaphoreGive(flash_mutex);

		uint32_t min_avail = available[0];
		for(uint8_t i = 1;i < 4;i++) {
			if(available[i] < min_avail) {
				min_avail = available[i];
			}
		}

		char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 37];
		generate_space_string(min_avail, logmsg, sizeof(STAT_AVAILABLE_FLASH) + 37);
		if(numbytes >= sizeof(logmsg) - 4) {
			memcpy(logstring, logmsg + 4, sizeof(logmsg) - 4);
		}
		else {
			memcpy(logstring, "\0", 1);
		}

		for(uint8_t i = 0;i < 4;i++) {
			char chipmsg[sizeof(STAT_AVAILABLE_FLASH) + 37];
			generate_space_string(available[i], chipmsg, sizeof(STAT_AVAILABLE_FLASH) + 37);
			log_message(chipmsg, -1);
		}
	}
}

void generate_space_string(uint32_t available, char *logstring, int numbytes) {
	uint8_t percent = (((uint64_t) available) * 100) / 536870912;
	if(available < 1024) {
    	char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 23];
    	snprintf(logmsg, sizeof(logmsg), STAT_AVAILABLE_FLASH "%" PRIu32 "B/512MB %u%%", available, percent);
    	if(numbytes >= sizeof(logmsg)) {
    		memcpy(logstring, logmsg, sizeof(logmsg));
    		return;
    	}
	}
	else if(available < 1048576UL) {
    	char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 37];
    	snprintf(logmsg, sizeof(logmsg), STAT_AVAILABLE_FLASH "%" PRIu32 "B: %" PRIu32 "KB/512MB %u%%", available, available >> 10, percent);
    	if(numbytes >= sizeof(logmsg)) {
    		memcpy(logstring, logmsg, sizeof(logmsg));
    		return;
    	}

	}
	else {
    	char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 32];
    	snprintf(logmsg, sizeof(logmsg), STAT_AVAILABLE_FLASH "%" PRIu32 "B: %" PRIu16 "MB/512MB %u%%", available, (uint16_t) (available >> 20), percent);
    	if(numbytes >= sizeof(logmsg)) {
    		memcpy(logstring, logmsg, sizeof(logmsg));
    		return;
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
    		for(uint8_t i = 0;i < 4;i++) {
        		finish_flash_write(&(flash_chips[i])); // Flush write buffer
            	erase_flash(&(flash_chips[i])); // Clear
    		}
    		xSemaphoreGive(flash_mutex);
    		log_message(STAT_CLEAR_FLASH, -1);
			Message dev_cmd_ack = {0};
			dev_cmd_ack.type = MSG_DEVICE_ACK;
			dev_cmd_ack.data.device_ack.board_id = BOARD_FR;
			dev_cmd_ack.data.device_ack.cmd_id = DEVICE_CMD_CLEAR_FLASH;
			strlcpy(dev_cmd_ack.data.device_ack.payload, STAT_CLEAR_FLASH + 4, sizeof(dev_cmd_ack.data.device_ack.payload));
  			if(send_msg_to_device(&dev_cmd_ack, 5) != 0) {
  				// Server not up, target device not connected, or txbuffer is full
  			}
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
		for(uint8_t i = 0;i < 4;i++) {
			finish_flash_write(&(flash_chips[i]));
		}
		xSemaphoreGive(flash_mutex);
	}
}

void flush_flash_log_for_reset() {
	if(xSemaphoreTake(flash_mutex, 500) == pdPASS) {
		for(uint8_t i = 0;i < 4;i++) {
			finish_flash_write(&(flash_chips[i]));
		}
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
