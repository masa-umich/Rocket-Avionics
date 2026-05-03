/*
 * udptelemetry.c
 *
 *  Created on: Dec 6, 2025
 *      Author: felix
 */

#include "udptelemetry.h"

struct netconn *telemudp_h = NULL;
SemaphoreHandle_t telemudp_mutex;

struct netconn *logudp_h = NULL;
SemaphoreHandle_t logudp_mutex;

uint8_t logging_enabled = 0;
SemaphoreHandle_t enable_mutex;

void switch_telem_logging(uint8_t enable) {
	if(xSemaphoreTake(enable_mutex, 2) == pdPASS) {
		logging_enabled = enable;
		xSemaphoreGive(enable_mutex);
		set_logging_state(enable);
	}
}

uint8_t telemetry_setup(uint8_t enabled) {
	logging_enabled = enabled;
	telemudp_mutex = xSemaphoreCreateMutex();
	logudp_mutex = xSemaphoreCreateMutex();
	enable_mutex = xSemaphoreCreateMutex();

    return 0;
}

void init_udp_telem() {
	if(xSemaphoreTake(telemudp_mutex, portMAX_DELAY) == pdPASS) {
		if(!telemudp_h) {
			telemudp_h = netconn_new(NETCONN_UDP);
			if(telemudp_h) {
				ip_set_option(telemudp_h->pcb.udp, SOF_BROADCAST);
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

	if(xSemaphoreTake(logudp_mutex, portMAX_DELAY) == pdPASS) {
		if(!logudp_h) {
			logudp_h = netconn_new(NETCONN_UDP);
			if(logudp_h) {
				ip_set_option(logudp_h->pcb.udp, SOF_BROADCAST);
				netconn_set_nonblocking(logudp_h, 1);
				netconn_bind(logudp_h, IP4_ADDR_ANY, ERROR_UDP_PORT);
			}
			else {
				// failed to create netconn
				log_message(FR_ERR_LOG_UDP_INIT, -1);
			}
		}
		else {
			log_message(FR_ERR_LOG_UDP_INIT, -1);
		}
		xSemaphoreGive(logudp_mutex);
	}
}

void deinit_udp_telem() {
	if(xSemaphoreTake(telemudp_mutex, portMAX_DELAY) == pdPASS) {
		netconn_close(telemudp_h);
		netconn_delete(telemudp_h);
		telemudp_h = NULL;
		xSemaphoreGive(telemudp_mutex);
	}

	if(xSemaphoreTake(logudp_mutex, portMAX_DELAY) == pdPASS) {
		netconn_close(logudp_h);
		netconn_delete(logudp_h);
		logudp_h = NULL;
		xSemaphoreGive(logudp_mutex);
	}
}

void TelemetrySend(void *argument) {
	for(;;) {
		struct netbuf *buf = NULL;
		if(xSemaphoreTake(telemudp_mutex, 5) == pdPASS) {
			if(telemudp_h) {
				err_t recv_err = netconn_recv(telemudp_h, &buf);
				if(recv_err == ERR_OK) {
					if(buf) {
						uint8_t msgbuf[256];
						uint16_t msg_len = netbuf_len(buf);
						if(msg_len > 256 || msg_len == 0) {
							log_message(FR_ERR_UDP_TELEM_RECV_SIZE_ERR, FR_ERR_TYPE_UDP_TELEM);
						}
						else {
							uint16_t ret = netbuf_copy(buf, msgbuf, msg_len);
							if(ret != 0 && ret == msg_len) {
								log_udp_telemetry(msgbuf, msg_len);
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

void LogListener(void *argument) {
	for(;;) {
		struct netbuf *buf = NULL;
		if(xSemaphoreTake(logudp_mutex, 5) == pdPASS) {
			if(logudp_h) {
				err_t recv_err = netconn_recv(logudp_h, &buf);
				if(recv_err == ERR_OK) {
					if(buf) {
						uint8_t msgbuf[256];
						uint16_t msg_len = netbuf_len(buf);
						if(msg_len > 256 || msg_len == 0) {
							log_message(FR_ERR_UDP_LOG_RECV_SIZE_ERR, FR_ERR_TYPE_UDP_TELEM);
						}
						else {
							uint16_t ret = netbuf_copy(buf, msgbuf, msg_len);
							if(ret != 0 && ret == msg_len) {
								log_udp_log(msgbuf, msg_len);
							}
						}
						netbuf_delete(buf);
					}
				}
			}
			else {
				// If not connected, delay to give cpu to other tasks
				xSemaphoreGive(logudp_mutex);
				osDelay(500);
				continue;
			}
			xSemaphoreGive(logudp_mutex);
		}
		osDelay(1);
	}
}

void log_udp_telemetry(uint8_t *message, uint16_t msg_len) {
	if(xSemaphoreTake(enable_mutex, 2) == pdPASS) {
		uint8_t enabled = logging_enabled;
		xSemaphoreGive(enable_mutex);
		if(enabled) {
			log_lmp_packet(message, msg_len);
		}
	}
}

void log_udp_log(uint8_t *message, uint16_t msg_len) {
	if(xSemaphoreTake(enable_mutex, 2) == pdPASS) {
		uint8_t enabled = logging_enabled;
		xSemaphoreGive(enable_mutex);
		if(enabled) {
			write_log_to_flash(message, msg_len);
		}
	}
}
