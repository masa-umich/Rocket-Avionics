/*
 * remote-config.c
 *
 *  Created on: Nov 9, 2025
 *      Author: felix
 */

#include "remote-config.h"

extern CRC_HandleTypeDef hcrc;

void *ftp_open(const char *fname, const char *mode, u8_t write) {
	if(strcmp(fname, "eeprom.bin") == 0) {
		prepare_eeprom_config();
		return write ? (void *) 0 : (void *) 1;
	}
	else if(strcmp(fname, "dump1.txt") == 0) {
		if(!write) {
			prepare_flash_dump();
		}
		return write ? (void *) 0 : (void *) (FLASH_BOTH_MARK + 0);
	}
	else if(strcmp(fname, "dump2.txt") == 0) {
		if(!write) {
			prepare_flash_dump();
		}
		return write ? (void *) 0 : (void *) (FLASH_BOTH_MARK + 1);
	}
	else if(strcmp(fname, "dump3.txt") == 0) {
		if(!write) {
			prepare_flash_dump();
		}
		return write ? (void *) 0 : (void *) (FLASH_BOTH_MARK + 2);
	}
	else if(strcmp(fname, "dump4.txt") == 0) {
		if(!write) {
			prepare_flash_dump();
		}
		return write ? (void *) 0 : (void *) (FLASH_BOTH_MARK + 3);
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
	else if(fd >= FLASH_BOTH_MARK && (fd - FLASH_BOTH_MARK < 4)) {
		finish_flash_dump();
	}
}

int ftp_read(void *handle, void *buf, int bytes) {
	uint32_t fd = (uint32_t) handle;
	if(fd == 1) {
		return eeprom_config_dump(buf, bytes);
	}
	else if(fd >= FLASH_BOTH_MARK && (fd - FLASH_BOTH_MARK < 4)) {
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
