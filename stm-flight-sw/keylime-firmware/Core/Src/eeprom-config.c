/*
 * eeprom-config.c
 *
 *  Created on: Nov 9, 2025
 *      Author: felix
 */

#include "eeprom-config.h"

eeprom_t eeprom_h = {0};
size_t eeprom_cursor = 0;

void timerRestart(TimerHandle_t xTimer) {
	reset_board();
}

void prepare_eeprom_config() {
	eeprom_cursor = 0;
}

void close_and_validate_config(CRC_HandleTypeDef *hcrc) {
	eeprom_cursor -= 4;
	if(eeprom_cursor < FR_EEPROM_LEN) {
		// too short
		log_message(ERR_TFTP_EERPOM_TOO_SHORT, -1);
		return;
	}
	// CRC check
	uint32_t sent_crc;
	eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, eeprom_cursor + FR_EEPROM_LEN, (uint8_t *) &sent_crc, 4);
	if(read_stat != EEPROM_OK) {
		// read error
		log_message(ERR_TFTP_EEPROM_READ_ERR, FR_ERR_TYPE_TFTP_EEPROM_READ);
		return;
	}
	HAL_CRC_Calculate(hcrc, NULL, 0); // reset calculation
	uint32_t calc_crc;
	int cursor = 0;
	do {
		int read_bytes = (eeprom_cursor - cursor < 64) ? eeprom_cursor - cursor : 64;
		uint8_t buf[read_bytes];
		read_stat = eeprom_read_mem(&eeprom_h, FR_EEPROM_LEN + cursor, buf, read_bytes);
		if(read_stat != EEPROM_OK) {
			// read error
			log_message(ERR_TFTP_EEPROM_READ_ERR, FR_ERR_TYPE_TFTP_EEPROM_READ);
			return;
		}
		calc_crc = HAL_CRC_Accumulate(hcrc, (uint32_t *) buf, read_bytes);
		cursor += read_bytes;
	} while(eeprom_cursor - cursor > 0);

	calc_crc ^= 0xFFFFFFFF; // Invert bits to follow crc32 standard

	if(sent_crc == calc_crc) {
		// Matches, copy contents into active section of eeprom
		cursor = 0;
		do {
			int read_bytes = (eeprom_cursor - cursor < 64) ? eeprom_cursor - cursor : 64;
			uint8_t buf[read_bytes];
			read_stat = eeprom_read_mem(&eeprom_h, FR_EEPROM_LEN + cursor, buf, read_bytes);
			if(read_stat != EEPROM_OK) {
				// copy read error
				log_message(ERR_TFTP_EEPROM_READ_ERR, FR_ERR_TYPE_TFTP_EEPROM_READ);
				return;
			}
			eeprom_status_t write_stat = eeprom_write_mem(&eeprom_h, cursor, buf, read_bytes);
			if(write_stat != EEPROM_OK) {
				// copy write error
				log_message(ERR_TFTP_EEPROM_WRITE_ERR, FR_ERR_TYPE_TFTP_EEPROM_WRITE);
				return;
			}
			cursor += read_bytes;
		} while(eeprom_cursor - cursor > 0);
		// eeprom successfully updated! restart board for new config to take effect
#ifdef RESTART_AFTER_CONFIG
		log_message(STAT_EEPROM_CONFIG_CHANGED EEPROM_CONFIG_RESTART, -1);
		TimerHandle_t resetTimer = xTimerCreate("restart", EEPROM_RESTART_DELAY_MS, pdFALSE, NULL, timerRestart);
		if(resetTimer) xTimerStart(resetTimer, 0);
#else
		log_message(STAT_EEPROM_CONFIG_CHANGED, -1);
#endif
	}
	else {
		// Mismatched CRC
		log_message(ERR_TFTP_EEPROM_BAD_CRC, -1);
	}
}

int eeprom_config_dump(void *buf, int bytes) {
	// Read from EEPROM
	int bytes_to_read = (eeprom_cursor + bytes > FR_EEPROM_LEN) ? FR_EEPROM_LEN - eeprom_cursor : bytes;
	if(bytes_to_read == 0) {
		return 0;
	}
	eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, eeprom_cursor, buf, bytes_to_read);
	if(read_stat != EEPROM_OK) {
		// eeprom read error
		log_message(ERR_TFTP_EEPROM_READ_ERR, FR_ERR_TYPE_TFTP_EEPROM_READ);
		return -1;
	}
	eeprom_cursor += bytes_to_read;
	return bytes_to_read;
}

int eeprom_config_write(struct pbuf *p) {
	// EEPROM
	struct pbuf *currbuf = p;
	do {
		eeprom_status_t write_stat = eeprom_write_mem(&eeprom_h, eeprom_cursor + FR_EEPROM_LEN, currbuf->payload, currbuf->len);
		if(write_stat != EEPROM_OK) {
			// eeprom write error
			log_message(ERR_TFTP_EEPROM_WRITE_ERR, FR_ERR_TYPE_TFTP_EEPROM_WRITE);
			return -1;
		}
		eeprom_cursor += currbuf->len;
		currbuf = currbuf->next;
	} while (currbuf != NULL);
	return p->tot_len;
}

uint8_t setup_eeprom(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *WC_GPIO_Port, uint16_t WC_Pin) {
	return eeprom_init(&eeprom_h, hi2c, WC_GPIO_Port, WC_Pin) == EEPROM_OK;
}

// Load board configuration from a buffer. Returns 0 on success, -1 on an eeprom error, -2 on invalid tc gains, -3 on invalid valve configuration values, and -4 on an invalid bay board number
int load_eeprom_config(EEPROM_conf_t *conf) {
	uint8_t buffer[FR_EEPROM_LEN];
	eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, 0, buffer, FR_EEPROM_LEN);
	if(read_stat != EEPROM_OK) {
		return -1;
	}

	IP4_ADDR(&(conf->flightcomputerIP), buffer[0], buffer[1], buffer[2], buffer[3]);
	IP4_ADDR(&(conf->flightrecorderIP), buffer[4], buffer[5], buffer[6], buffer[7]);

	int ret = 0;
	return ret;
}

void load_eeprom_defaults(EEPROM_conf_t *conf) {
	IP4_ADDR(&(conf->flightcomputerIP), FR_EEPROM_FCIP_DEFAULT_1, FR_EEPROM_FCIP_DEFAULT_2, FR_EEPROM_FCIP_DEFAULT_3, FR_EEPROM_FCIP_DEFAULT_4);
	IP4_ADDR(&(conf->flightrecorderIP), FR_EEPROM_FRIP_DEFAULT_1, FR_EEPROM_FRIP_DEFAULT_2, FR_EEPROM_FRIP_DEFAULT_3, FR_EEPROM_FRIP_DEFAULT_4);
}
