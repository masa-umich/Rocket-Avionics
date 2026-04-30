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
#include "ip4_addr.h"
#include "log_errors.h"
#include "timers.h"

#define EEPROM_LOGGING_PARAM_ADDR		(uint16_t)0x2000

typedef struct {
	ip4_addr_t flightcomputerIP;
	ip4_addr_t flightrecorderIP;
} EEPROM_conf_t;

int get_logging_state(uint8_t blocking);

void set_logging_state(uint8_t state);

void prepare_eeprom_config();

void close_and_validate_config(CRC_HandleTypeDef *hcrc);

int eeprom_config_dump(void *buf, int bytes);

int eeprom_config_write(struct pbuf *p);

uint8_t setup_eeprom(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *WC_GPIO_Port, uint16_t WC_Pin);

int load_eeprom_config(EEPROM_conf_t *conf);

void load_eeprom_defaults(EEPROM_conf_t *conf);

#endif /* INC_EEPROM_CONFIG_H_ */
