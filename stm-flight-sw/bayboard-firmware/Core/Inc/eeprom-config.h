/*
 * eeprom-config.h
 *
 *  Created on: Nov 9, 2025
 *      Author: felix
 */

#ifndef INC_EEPROM_CONFIG_H_
#define INC_EEPROM_CONFIG_H_

#include "main.h"
#include "M24256E.h"
#include "utils.h"
#include "VLVs.h"
#include "ip4_addr.h"
#include "log_errors.h"
#include "timers.h"

typedef struct {
	PT_t *pt1;
	PT_t *pt2;
	PT_t *pt3;
	PT_t *pt4;
	PT_t *pt5;
	PT_t *pt6;
	PT_t *pt7;
	PT_t *pt8;
	PT_t *pt9;
	PT_t *pt10;

	uint8_t tc1_gain;
	uint8_t tc2_gain;
	uint8_t tc3_gain;
	uint8_t tc4_gain;
	uint8_t tc5_gain;
	uint8_t tc6_gain;

	VLV_Voltage vlv1_v;
	uint8_t vlv1_en;
	VLV_Voltage vlv2_v;
	uint8_t vlv2_en;
	VLV_Voltage vlv3_v;
	uint8_t vlv3_en;
	VLV_Voltage vlv4_v;
	uint8_t vlv4_en;
	VLV_Voltage vlv5_v;
	uint8_t vlv5_en;

	ip4_addr_t flightcomputerIP;
	ip4_addr_t bayboardIP;
} EEPROM_conf_t;

void prepare_eeprom_config();

void close_and_validate_config(CRC_HandleTypeDef *hcrc);

int eeprom_config_dump(void *buf, int bytes);

int eeprom_config_write(struct pbuf *p);

uint8_t setup_eeprom(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *WC_GPIO_Port, uint16_t WC_Pin);

int load_eeprom_config(EEPROM_conf_t *conf, uint8_t *bb);

void load_eeprom_defaults(EEPROM_conf_t *conf, uint8_t *bb);

#endif /* INC_EEPROM_CONFIG_H_ */
