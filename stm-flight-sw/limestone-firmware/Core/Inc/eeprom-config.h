/*
 * eeprom-config.h
 *
 *  Created on: Oct 21, 2025
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

#define EEPROM_BOOT_PARAMS_ADDR		(uint16_t)0x2000 // stay clear of board config
#define EEPROM_RADIO_PARAM_ADDR		EEPROM_BOOT_PARAMS_ADDR - 1

typedef struct {
	PT_t *pt1;
	PT_t *pt2;
	PT_t *pt3;
	PT_t *pt4;
	PT_t *pt5;

	uint8_t tc1_gain;
	uint8_t tc2_gain;
	uint8_t tc3_gain;

	VLV_Voltage vlv1_v;
	uint8_t vlv1_en;
	VLV_Voltage vlv2_v;
	uint8_t vlv2_en;
	VLV_Voltage vlv3_v;
	uint8_t vlv3_en;

	ip4_addr_t limewireIP;
	ip4_addr_t flightcomputerIP;
	ip4_addr_t bayboard1IP;
	ip4_addr_t bayboard2IP;
	ip4_addr_t bayboard3IP;
	ip4_addr_t flightrecordIP;

	uint8_t fuel_mpv_index;
	uint8_t ox_mpv_index;
	uint8_t pilot_para_index;
	uint8_t	drogue_para_index;
	uint8_t main_para_index;
} EEPROM_conf_t;

int get_radio_state(uint8_t blocking);

void set_radio_state(uint8_t state);

uint8_t write_autosequence_params(void * buf, size_t len);

uint8_t read_autosequence_params(void * buf, size_t len);

void prepare_eeprom_config();

void close_and_validate_config(CRC_HandleTypeDef *hcrc);

int eeprom_config_dump(void *buf, int bytes);

int eeprom_config_write(struct pbuf *p);

uint8_t setup_eeprom(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *WC_GPIO_Port, uint16_t WC_Pin);

int load_eeprom_config(EEPROM_conf_t *conf);

void load_eeprom_defaults(EEPROM_conf_t *conf);

#endif /* INC_EEPROM_CONFIG_H_ */
