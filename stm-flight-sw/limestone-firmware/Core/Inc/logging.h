/*
 * logging.h
 *
 *  Created on: Oct 19, 2025
 *      Author: felix
 */

#ifndef INC_LOGGING_H_
#define INC_LOGGING_H_

#include "main.h"
#include "W25N04KV.h"
#include "base64.h"
#include "queue.h"
#include "time-sync.h"
#include "system-state.h"
#include "api.h"
#include "lwip/udp.h"
#include "ip4_addr.h"
#include "log_errors.h"

typedef struct {
	uint8_t *content;
	size_t len;
} errormsg_t;

uint8_t write_ascii_to_flash(const char *msgtext, size_t msglen, uint8_t type);

void init_network_logging(uint8_t reinit, ip4_addr_t ipaddr);

void deinit_network_logging();

void logging_setup();

uint8_t init_flash_logging(SPI_HandleTypeDef * hspi, GPIO_TypeDef *CS_GPIO_Port, uint16_t CS_GPIO_Pin);

void log_telemetry();

void log_valve_states();

void handle_logging();

void prepare_flash_dump();

void finish_flash_dump();

int dump_flash(uint32_t fd, void *buf, int bytes);

uint8_t write_ascii_to_flash(const char *msgtext, size_t msglen, uint8_t type);

uint8_t write_raw_to_flash(uint8_t *writebuf, size_t msglen);

uint8_t log_message(const char *msgtext, int msgtype);

uint8_t log_peri_message(const char *msgtext, int msgtype);

void send_flash_full();

int log_lmp_packet(uint8_t *buf, size_t buflen);

void send_udp_online(ip4_addr_t * ip);

void log_flash_storage(char *logstring, int numbytes);

void clear_flash();

void handle_flash_clearing();

uint8_t is_net_logging_up();

void FlashClearTask(void *argument);

void refresh_log_timers();

void flush_flash_log();

#endif /* INC_LOGGING_H_ */
