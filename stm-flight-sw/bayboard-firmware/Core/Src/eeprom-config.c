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
	if(eeprom_cursor < BB_EEPROM_LEN) {
		// too short
		log_message(ERR_TFTP_EERPOM_TOO_SHORT, -1);
		return;
	}
	// CRC check
	uint32_t sent_crc;
	eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, eeprom_cursor + BB_EEPROM_LEN, (uint8_t *) &sent_crc, 4);
	if(read_stat != EEPROM_OK) {
		// read error
		log_message(ERR_TFTP_EEPROM_READ_ERR, BB_ERR_TYPE_TFTP_EEPROM_READ);
		return;
	}
	HAL_CRC_Calculate(hcrc, NULL, 0); // reset calculation
	uint32_t calc_crc;
	int cursor = 0;
	do {
		int read_bytes = (eeprom_cursor - cursor < 64) ? eeprom_cursor - cursor : 64;
		uint8_t buf[read_bytes];
		read_stat = eeprom_read_mem(&eeprom_h, BB_EEPROM_LEN + cursor, buf, read_bytes);
		if(read_stat != EEPROM_OK) {
			// read error
			log_message(ERR_TFTP_EEPROM_READ_ERR, BB_ERR_TYPE_TFTP_EEPROM_READ);
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
			read_stat = eeprom_read_mem(&eeprom_h, BB_EEPROM_LEN + cursor, buf, read_bytes);
			if(read_stat != EEPROM_OK) {
				// copy read error
				log_message(ERR_TFTP_EEPROM_READ_ERR, BB_ERR_TYPE_TFTP_EEPROM_READ);
				return;
			}
			eeprom_status_t write_stat = eeprom_write_mem(&eeprom_h, cursor, buf, read_bytes);
			if(write_stat != EEPROM_OK) {
				// copy write error
				log_message(ERR_TFTP_EEPROM_WRITE_ERR, BB_ERR_TYPE_TFTP_EEPROM_WRITE);
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
	int bytes_to_read = (eeprom_cursor + bytes > BB_EEPROM_LEN) ? BB_EEPROM_LEN - eeprom_cursor : bytes;
	if(bytes_to_read == 0) {
		return 0;
	}
	eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, eeprom_cursor, buf, bytes_to_read);
	if(read_stat != EEPROM_OK) {
		// eeprom read error
		log_message(ERR_TFTP_EEPROM_READ_ERR, BB_ERR_TYPE_TFTP_EEPROM_READ);
		return -1;
	}
	eeprom_cursor += bytes_to_read;
	return bytes_to_read;
}

int eeprom_config_write(struct pbuf *p) {
	// EEPROM
	struct pbuf *currbuf = p;
	do {
		eeprom_status_t write_stat = eeprom_write_mem(&eeprom_h, eeprom_cursor + BB_EEPROM_LEN, currbuf->payload, currbuf->len);
		if(write_stat != EEPROM_OK) {
			// eeprom write error
			log_message(ERR_TFTP_EEPROM_WRITE_ERR, BB_ERR_TYPE_TFTP_EEPROM_WRITE);
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
int load_eeprom_config(EEPROM_conf_t *conf, uint8_t *bb) {
	uint8_t buffer[BB_EEPROM_LEN];
	eeprom_status_t read_stat = eeprom_read_mem(&eeprom_h, 0, buffer, BB_EEPROM_LEN);
	if(read_stat != EEPROM_OK) {
		return -1;
	}

	*bb = buffer[0];

	memcpy(&(conf->pt1->zero_V), 		buffer + 1, 4);
	memcpy(&(conf->pt1->pres_range), 	buffer + 5, 4);
	memcpy(&(conf->pt1->max_V), 		buffer + 9, 4);
	memcpy(&(conf->pt2->zero_V), 		buffer + 13, 4);
	memcpy(&(conf->pt2->pres_range), 	buffer + 17, 4);
	memcpy(&(conf->pt2->max_V), 		buffer + 21, 4);
	memcpy(&(conf->pt3->zero_V), 		buffer + 25, 4);
	memcpy(&(conf->pt3->pres_range), 	buffer + 29, 4);
	memcpy(&(conf->pt3->max_V), 		buffer + 33, 4);
	memcpy(&(conf->pt4->zero_V), 		buffer + 37, 4);
	memcpy(&(conf->pt4->pres_range), 	buffer + 41, 4);
	memcpy(&(conf->pt4->max_V), 		buffer + 45, 4);
	memcpy(&(conf->pt5->zero_V), 		buffer + 49, 4);
	memcpy(&(conf->pt5->pres_range), 	buffer + 53, 4);
	memcpy(&(conf->pt5->max_V), 		buffer + 57, 4);
	memcpy(&(conf->pt6->zero_V), 		buffer + 61, 4);
	memcpy(&(conf->pt6->pres_range),	buffer + 65, 4);
	memcpy(&(conf->pt6->max_V), 		buffer + 69, 4);
	memcpy(&(conf->pt7->zero_V), 		buffer + 73, 4);
	memcpy(&(conf->pt7->pres_range), 	buffer + 77, 4);
	memcpy(&(conf->pt7->max_V), 		buffer + 81, 4);
	memcpy(&(conf->pt8->zero_V), 		buffer + 85, 4);
	memcpy(&(conf->pt8->pres_range), 	buffer + 89, 4);
	memcpy(&(conf->pt8->max_V), 		buffer + 93, 4);
	memcpy(&(conf->pt9->zero_V), 		buffer + 97, 4);
	memcpy(&(conf->pt9->pres_range), 	buffer + 101, 4);
	memcpy(&(conf->pt9->max_V), 		buffer + 105, 4);
	memcpy(&(conf->pt10->zero_V), 		buffer + 109, 4);
	memcpy(&(conf->pt10->pres_range), 	buffer + 113, 4);
	memcpy(&(conf->pt10->max_V), 		buffer + 117, 4);

	conf->tc1_gain = buffer[121] << 1;
	conf->tc2_gain = buffer[122] << 1;
	conf->tc3_gain = buffer[123] << 1;
	conf->tc4_gain = buffer[124] << 1;
	conf->tc5_gain = buffer[125] << 1;
	conf->tc6_gain = buffer[126] << 1;

	conf->vlv1_v = buffer[127];
	conf->vlv1_en = buffer[128];
	conf->vlv2_v = buffer[129];
	conf->vlv2_en = buffer[130];
	conf->vlv3_v = buffer[131];
	conf->vlv3_en = buffer[132];
	conf->vlv4_v = buffer[133];
	conf->vlv4_en = buffer[134];
	conf->vlv5_v = buffer[135];
	conf->vlv5_en = buffer[136];

	IP4_ADDR(&(conf->flightcomputerIP), buffer[137], buffer[138], buffer[139], buffer[140]);
	IP4_ADDR(&(conf->bayboardIP), buffer[141], buffer[142], buffer[143], buffer[144]);

	int ret = 0;
	if(conf->tc1_gain > 0x0E || conf->tc2_gain > 0x0E || conf->tc3_gain > 0x0E || conf->tc4_gain > 0x0E || conf->tc5_gain > 0x0E || conf->tc6_gain > 0x0E) {
		conf->tc1_gain = BB_EEPROM_TC_GAIN_DEFAULT;
		conf->tc2_gain = BB_EEPROM_TC_GAIN_DEFAULT;
		conf->tc3_gain = BB_EEPROM_TC_GAIN_DEFAULT;
		conf->tc4_gain = BB_EEPROM_TC_GAIN_DEFAULT;
		conf->tc5_gain = BB_EEPROM_TC_GAIN_DEFAULT;
		conf->tc6_gain = BB_EEPROM_TC_GAIN_DEFAULT;
		ret = -2;
	}

	if(conf->vlv1_v > 0x01 || conf->vlv1_en > 0x01 || conf->vlv2_v > 0x01 || conf->vlv2_en > 0x01 || conf->vlv3_v > 0x01 || conf->vlv3_en > 0x01 || conf->vlv4_v > 0x01 || conf->vlv4_en > 0x01 || conf->vlv5_v > 0x01 || conf->vlv5_en > 0x01) {
		conf->vlv1_v = BB_EEPROM_VLV_VOL_DEFAULT;
		conf->vlv1_en = BB_EEPROM_VLV_EN_DEFAULT;
		conf->vlv2_v = BB_EEPROM_VLV_VOL_DEFAULT;
		conf->vlv2_en = BB_EEPROM_VLV_EN_DEFAULT;
		conf->vlv3_v = BB_EEPROM_VLV_VOL_DEFAULT;
		conf->vlv3_en = BB_EEPROM_VLV_EN_DEFAULT;
		conf->vlv4_v = BB_EEPROM_VLV_VOL_DEFAULT;
		conf->vlv4_en = BB_EEPROM_VLV_EN_DEFAULT;
		conf->vlv5_v = BB_EEPROM_VLV_VOL_DEFAULT;
		conf->vlv5_en = BB_EEPROM_VLV_EN_DEFAULT;
		ret = -3;
	}

	if(*bb == 0 || *bb > 3) {
		*bb = 1;
		ret = -4;
	}

	return ret;
}

void load_eeprom_defaults(EEPROM_conf_t *conf, uint8_t *bb) {
	*bb = 1;
	conf->pt1->zero_V 		= BB_EEPROM_PT_ZERO_DEFAULT;
	conf->pt1->pres_range 	= BB_EEPROM_PT_RANGE_DEFAULT;
	conf->pt1->max_V 		= BB_EEPROM_PT_MAX_DEFAULT;
	conf->pt2->zero_V 		= BB_EEPROM_PT_ZERO_DEFAULT;
	conf->pt2->pres_range 	= BB_EEPROM_PT_RANGE_DEFAULT;
	conf->pt2->max_V 		= BB_EEPROM_PT_MAX_DEFAULT;
	conf->pt3->zero_V 		= BB_EEPROM_PT_ZERO_DEFAULT;
	conf->pt3->pres_range 	= BB_EEPROM_PT_RANGE_DEFAULT;
	conf->pt3->max_V 		= BB_EEPROM_PT_MAX_DEFAULT;
	conf->pt4->zero_V 		= BB_EEPROM_PT_ZERO_DEFAULT;
	conf->pt4->pres_range 	= BB_EEPROM_PT_RANGE_DEFAULT;
	conf->pt4->max_V 		= BB_EEPROM_PT_MAX_DEFAULT;
	conf->pt5->zero_V 		= BB_EEPROM_PT_ZERO_DEFAULT;
	conf->pt5->pres_range 	= BB_EEPROM_PT_RANGE_DEFAULT;
	conf->pt5->max_V 		= BB_EEPROM_PT_MAX_DEFAULT;
	conf->pt6->zero_V 		= BB_EEPROM_PT_ZERO_DEFAULT;
	conf->pt6->pres_range 	= BB_EEPROM_PT_RANGE_DEFAULT;
	conf->pt6->max_V 		= BB_EEPROM_PT_MAX_DEFAULT;
	conf->pt7->zero_V 		= BB_EEPROM_PT_ZERO_DEFAULT;
	conf->pt7->pres_range 	= BB_EEPROM_PT_RANGE_DEFAULT;
	conf->pt7->max_V 		= BB_EEPROM_PT_MAX_DEFAULT;
	conf->pt8->zero_V 		= BB_EEPROM_PT_ZERO_DEFAULT;
	conf->pt8->pres_range 	= BB_EEPROM_PT_RANGE_DEFAULT;
	conf->pt8->max_V 		= BB_EEPROM_PT_MAX_DEFAULT;
	conf->pt9->zero_V 		= BB_EEPROM_PT_ZERO_DEFAULT;
	conf->pt9->pres_range 	= BB_EEPROM_PT_RANGE_DEFAULT;
	conf->pt9->max_V 		= BB_EEPROM_PT_MAX_DEFAULT;
	conf->pt10->zero_V 		= BB_EEPROM_PT_ZERO_DEFAULT;
	conf->pt10->pres_range 	= BB_EEPROM_PT_RANGE_DEFAULT;
	conf->pt10->max_V 		= BB_EEPROM_PT_MAX_DEFAULT;

	conf->tc1_gain = BB_EEPROM_TC_GAIN_DEFAULT;
	conf->tc2_gain = BB_EEPROM_TC_GAIN_DEFAULT;
	conf->tc3_gain = BB_EEPROM_TC_GAIN_DEFAULT;
	conf->tc4_gain = BB_EEPROM_TC_GAIN_DEFAULT;
	conf->tc5_gain = BB_EEPROM_TC_GAIN_DEFAULT;
	conf->tc6_gain = BB_EEPROM_TC_GAIN_DEFAULT;

	conf->vlv1_v = BB_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv1_en = BB_EEPROM_VLV_EN_DEFAULT;
	conf->vlv2_v = BB_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv2_en = BB_EEPROM_VLV_EN_DEFAULT;
	conf->vlv3_v = BB_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv3_en = BB_EEPROM_VLV_EN_DEFAULT;
	conf->vlv4_v = BB_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv4_en = BB_EEPROM_VLV_EN_DEFAULT;
	conf->vlv5_v = BB_EEPROM_VLV_VOL_DEFAULT;
	conf->vlv5_en = BB_EEPROM_VLV_EN_DEFAULT;

	IP4_ADDR(&(conf->flightcomputerIP), BB_EEPROM_FCIP_DEFAULT_1, BB_EEPROM_FCIP_DEFAULT_2, BB_EEPROM_FCIP_DEFAULT_3, BB_EEPROM_FCIP_DEFAULT_4);
	IP4_ADDR(&(conf->bayboardIP), BB_EEPROM_BBIP_DEFAULT_1, BB_EEPROM_BBIP_DEFAULT_2, BB_EEPROM_BBIP_DEFAULT_3, BB_EEPROM_BBIP_DEFAULT_4);
}
