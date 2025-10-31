/*
 * remote-config.c
 *
 *  Created on: Oct 21, 2025
 *      Author: felix
 */

#include "remote-config.h"

extern CRC_HandleTypeDef hcrc;

void *ftp_open(const char *fname, const char *mode, u8_t write) {
	if(strcmp(fname, "eeprom.bin") == 0) {
		prepare_eeprom_config();
		return (void *) 1; // 1 refers to eeprom
	}
	else if(strcmp(fname, "messages.txt") == 0) {
		if(!write) {
			prepare_flash_dump();
		}
		return write ? (void *) 0 : (void *) FLASH_MSG_MARK; // FLASH_MSG_MARK refers to status/error messages. Only reading is valid
	}
	else if(strcmp(fname, "telemetry.bin") == 0) {
		if(!write) {
			prepare_flash_dump();
		}
		return write ? (void *) 0 : (void *) FLASH_TELEM_MARK; // FLASH_TELEM_MARK refers to telemetry and valve state LMP dump. Only reading is valid
	}
	else {
		return (void *) 0;
	}
}

void ftp_close(void *handle) {
	uint32_t fd = (uint32_t) handle;
	if(fd == 1) {
		close_and_validate_config(&hcrc);
	}
	else if(fd == FLASH_MSG_MARK || fd == FLASH_TELEM_MARK) {
		finish_flash_dump();
	}
}

int ftp_read(void *handle, void *buf, int bytes) {
	uint32_t fd = (uint32_t) handle;
	if(fd == 1) {
		return eeprom_config_dump(buf, bytes);
	}
	else if(fd == FLASH_MSG_MARK || fd == FLASH_TELEM_MARK) {
		return dump_flash(fd, buf, bytes);
	}
	else {
		return -1;
	}
}

int ftp_write(void *handle, struct pbuf *p) {
	uint32_t fd = (uint32_t) handle;
	if(fd == 1) {
		return eeprom_config_write(p);
	}
	else {
		return -1;
	}
}

const struct tftp_context my_tftp_ctx = {
    .open = ftp_open,
    .close = ftp_close,
    .read = ftp_read,
    .write = ftp_write
};
