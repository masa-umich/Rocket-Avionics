/*
 * remote-config.h
 *
 *  Created on: Nov 9, 2025
 *      Author: felix
 */

#ifndef INC_REMOTE_CONFIG_H_
#define INC_REMOTE_CONFIG_H_

#include "main.h"
#include "logging.h"
#include "eeprom-config.h"
#include "tftp_server.h"
#include "log_errors.h"

void *ftp_open(const char *fname, const char *mode, u8_t write);

void ftp_close(void *handle);

int ftp_read(void *handle, void *buf, int bytes);

int ftp_write(void *handle, struct pbuf *p);

extern const struct tftp_context my_tftp_ctx;

#endif /* INC_REMOTE_CONFIG_H_ */
