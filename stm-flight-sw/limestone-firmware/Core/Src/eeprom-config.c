/*
 * eeprom-config.c
 *
 *  Created on: Oct 21, 2025
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
	if(eeprom_cursor < FC_EEPROM_LEN) {
		// too short
		log_message(ERR_TFTP_EERPOM_TOO_SHORT, -1);
		return;
	}
	// CRC check
	uint32_t sent_crc;
	eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, eeprom_cursor + FC_EEPROM_LEN, (uint8_t *) &sent_crc, 4);
	if(read_stat != EEPROM_OK) {
		// read error
		log_message(ERR_TFTP_EEPROM_READ_ERR, FC_ERR_TYPE_TFTP_EEPROM_READ);
		return;
	}
	HAL_CRC_Calculate(hcrc, NULL, 0); // reset calculation
	uint32_t calc_crc;
	int cursor = 0;
	do {
		int read_bytes = (eeprom_cursor - cursor < 64) ? eeprom_cursor - cursor : 64;
		uint8_t buf[read_bytes];
		read_stat = eeprom_read_mem(&eeprom_h, FC_EEPROM_LEN + cursor, buf, read_bytes);
		if(read_stat != EEPROM_OK) {
			// read error
			log_message(ERR_TFTP_EEPROM_READ_ERR, FC_ERR_TYPE_TFTP_EEPROM_READ);
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
			read_stat = eeprom_read_mem(&eeprom_h, FC_EEPROM_LEN + cursor, buf, read_bytes);
			if(read_stat != EEPROM_OK) {
				// copy read error
				log_message(ERR_TFTP_EEPROM_READ_ERR, FC_ERR_TYPE_TFTP_EEPROM_READ);
				return;
			}
			eeprom_status_t write_stat = eeprom_write_mem(&eeprom_h, cursor, buf, read_bytes);
			if(write_stat != EEPROM_OK) {
				// copy write error
				log_message(ERR_TFTP_EEPROM_WRITE_ERR, FC_ERR_TYPE_TFTP_EEPROM_WRITE);
				return;
			}
			cursor += read_bytes;
		} while(eeprom_cursor - cursor > 0);
		// eeprom successfully updated! restart board for new config to take effect
#ifdef RESTART_AFTER_CONFIG
		log_message(STAT_EEPROM_CONFIG_CHANGED EEPROM_CONFIG_RESTART, -1);
		xTimerCreate("restart", EEPROM_RESTART_DELAY_MS, pdFALSE, NULL, timerRestart);
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
	int bytes_to_read = (eeprom_cursor + bytes > FC_EEPROM_LEN) ? FC_EEPROM_LEN - eeprom_cursor : bytes;
	if(bytes_to_read == 0) {
		return 0;
	}
	eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, eeprom_cursor, buf, bytes_to_read);
	if(read_stat != EEPROM_OK) {
		// eeprom read error
		log_message(ERR_TFTP_EEPROM_READ_ERR, FC_ERR_TYPE_TFTP_EEPROM_READ);
		return -1;
	}
	eeprom_cursor += bytes_to_read;
	return bytes_to_read;
}

int eeprom_config_write(struct pbuf *p) {
	struct pbuf *currbuf = p;
	do {
		eeprom_status_t write_stat = eeprom_write_mem(&eeprom_h, eeprom_cursor + FC_EEPROM_LEN, currbuf->payload, currbuf->len);
		if(write_stat != EEPROM_OK) {
			// eeprom write error
			log_message(ERR_TFTP_EEPROM_WRITE_ERR, FC_ERR_TYPE_TFTP_EEPROM_WRITE);
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

// Load board configuration from a buffer. Returns 0 on success, -1 on an eeprom error, -2 on invalid tc gains, and -3 on invalid valve configuration values
int load_eeprom_config(EEPROM_conf_t *conf) {
	uint8_t buffer[FC_EEPROM_LEN];
	eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, 0, buffer, FC_EEPROM_LEN);
	if(read_stat != EEPROM_OK) {
		return -1;
	}
	memcpy(&(conf->pt1->zero_V), buffer, 4);
	memcpy(&(conf->pt1->pres_range), buffer + 4, 4);
	memcpy(&(conf->pt1->max_V), buffer + 8, 4);
	memcpy(&(conf->pt2->zero_V), buffer + 12, 4);
	memcpy(&(conf->pt2->pres_range), buffer + 16, 4);
	memcpy(&(conf->pt2->max_V), buffer + 20, 4);
	memcpy(&(conf->pt3->zero_V), buffer + 24, 4);
	memcpy(&(conf->pt3->pres_range), buffer + 28, 4);
	memcpy(&(conf->pt3->max_V), buffer + 32, 4);
	memcpy(&(conf->pt4->zero_V), buffer + 36, 4);
	memcpy(&(conf->pt4->pres_range), buffer + 40, 4);
	memcpy(&(conf->pt4->max_V), buffer + 44, 4);
	memcpy(&(conf->pt5->zero_V), buffer + 48, 4);
	memcpy(&(conf->pt5->pres_range), buffer + 52, 4);
	memcpy(&(conf->pt5->max_V), buffer + 56, 4);

	conf->tc1_gain = buffer[60] << 1;
	conf->tc2_gain = buffer[61] << 1;
	conf->tc3_gain = buffer[62] << 1;

	conf->vlv1_v = buffer[63];
	conf->vlv1_en = buffer[64];
	conf->vlv2_v = buffer[65];
	conf->vlv2_en = buffer[66];
	conf->vlv3_v = buffer[67];
	conf->vlv3_en = buffer[68];

	IP4_ADDR(&(conf->limewireIP), buffer[69], buffer[70], buffer[71], buffer[72]);
	IP4_ADDR(&(conf->flightcomputerIP), buffer[73], buffer[74], buffer[75], buffer[76]);
	IP4_ADDR(&(conf->bayboard1IP), buffer[77], buffer[78], buffer[79], buffer[80]);
	IP4_ADDR(&(conf->bayboard2IP), buffer[81], buffer[82], buffer[83], buffer[84]);
	IP4_ADDR(&(conf->bayboard3IP), buffer[85], buffer[86], buffer[87], buffer[88]);
	IP4_ADDR(&(conf->flightrecordIP), buffer[89], buffer[90], buffer[91], buffer[92]);

	int ret = 0;
	if(conf->tc1_gain > 0x0E || conf->tc2_gain > 0x0E || conf->tc3_gain > 0x0E) {
		conf->tc1_gain = FC_EEPROM_TC_GAIN_DEFAULT;
		conf->tc2_gain = FC_EEPROM_TC_GAIN_DEFAULT;
		conf->tc3_gain = FC_EEPROM_TC_GAIN_DEFAULT;
		ret = -2;
	}

	if(conf->vlv1_v > 0x01 || conf->vlv1_en > 0x01 || conf->vlv2_v > 0x01 || conf->vlv2_en > 0x01 || conf->vlv3_v > 0x01 || conf->vlv3_en > 0x01) {
		conf->vlv1_v = FC_EEPROM_VLV_VOL_DEFAULT;
		conf->vlv1_en = FC_EEPROM_VLV_EN_DEFAULT;
		conf->vlv2_v = FC_EEPROM_VLV_VOL_DEFAULT;
		conf->vlv2_en = FC_EEPROM_VLV_EN_DEFAULT;
		conf->vlv3_v = FC_EEPROM_VLV_VOL_DEFAULT;
		conf->vlv3_en = FC_EEPROM_VLV_EN_DEFAULT;
		ret = -3;
	}
	return ret;
}

void load_eeprom_defaults(EEPROM_conf_t *conf) {
	conf->pt1->zero_V = FC_EEPROM_PT_ZERO_DEFAULT;
	conf->pt1->pres_range = FC_EEPROM_PT_RANGE_DEFAULT;
	conf->pt1->max_V = FC_EEPROM_PT_MAX_DEFAULT;
	conf->pt2->zero_V = FC_EEPROM_PT_ZERO_DEFAULT;
	conf->pt2->pres_range = FC_EEPROM_PT_RANGE_DEFAULT;
	conf->pt2->max_V = FC_EEPROM_PT_MAX_DEFAULT;
	conf->pt3->zero_V = FC_EEPROM_PT_ZERO_DEFAULT;
	conf->pt3->pres_range = FC_EEPROM_PT_RANGE_DEFAULT;
	conf->pt3->max_V = FC_EEPROM_PT_MAX_DEFAULT;
	conf->pt4->zero_V = FC_EEPROM_PT_ZERO_DEFAULT;
	conf->pt4->pres_range = FC_EEPROM_PT_RANGE_DEFAULT;
	conf->pt4->max_V = FC_EEPROM_PT_MAX_DEFAULT;
	conf->pt5->zero_V = FC_EEPROM_PT_ZERO_DEFAULT;
	conf->pt5->pres_range = FC_EEPROM_PT_RANGE_DEFAULT;
	conf->pt5->max_V = FC_EEPROM_PT_MAX_DEFAULT;

	conf->tc1_gain = FC_EEPROM_TC_GAIN_DEFAULT;
	conf->tc2_gain = FC_EEPROM_TC_GAIN_DEFAULT;
	conf->tc3_gain = FC_EEPROM_TC_GAIN_DEFAULT;

	conf->vlv1_v = FC_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv1_en = FC_EEPROM_VLV_EN_DEFAULT;
	conf->vlv2_v = FC_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv2_en = FC_EEPROM_VLV_EN_DEFAULT;
	conf->vlv3_v = FC_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv3_en = FC_EEPROM_VLV_EN_DEFAULT;

	IP4_ADDR(&(conf->limewireIP), FC_EEPROM_LIMEWIREIP_DEFAULT_1, FC_EEPROM_LIMEWIREIP_DEFAULT_2, FC_EEPROM_LIMEWIREIP_DEFAULT_3, FC_EEPROM_LIMEWIREIP_DEFAULT_4);
	IP4_ADDR(&(conf->flightcomputerIP), FC_EEPROM_FCIP_DEFAULT_1, FC_EEPROM_FCIP_DEFAULT_2, FC_EEPROM_FCIP_DEFAULT_3, FC_EEPROM_FCIP_DEFAULT_4);
	IP4_ADDR(&(conf->bayboard1IP), FC_EEPROM_BB1IP_DEFAULT_1, FC_EEPROM_BB1IP_DEFAULT_2, FC_EEPROM_BB1IP_DEFAULT_3, FC_EEPROM_BB1IP_DEFAULT_4);
	IP4_ADDR(&(conf->bayboard2IP), FC_EEPROM_BB2IP_DEFAULT_1, FC_EEPROM_BB2IP_DEFAULT_2, FC_EEPROM_BB2IP_DEFAULT_3, FC_EEPROM_BB2IP_DEFAULT_4);
	IP4_ADDR(&(conf->bayboard3IP), FC_EEPROM_BB3IP_DEFAULT_1, FC_EEPROM_BB3IP_DEFAULT_2, FC_EEPROM_BB3IP_DEFAULT_3, FC_EEPROM_BB3IP_DEFAULT_4);
	IP4_ADDR(&(conf->flightrecordIP), FC_EEPROM_FRIP_DEFAULT_1, FC_EEPROM_FRIP_DEFAULT_2, FC_EEPROM_FRIP_DEFAULT_3, FC_EEPROM_FRIP_DEFAULT_4);
}
