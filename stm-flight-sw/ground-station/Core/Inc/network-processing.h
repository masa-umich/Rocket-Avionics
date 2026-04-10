/*
 * network-processing.h
 *
 *  Created on: Oct 29, 2025
 *      Author: felix
 */

#ifndef INC_NETWORK_PROCESSING_H_
#define INC_NETWORK_PROCESSING_H_

#include "main.h"
#include "logging.h"
#include "messages.h"
#include "server.h"
#include "log_errors.h"
#include "utils.h"
#include "eeprom-config.h"

extern EEPROM_conf_t loaded_config;

void ProcessPackets(void *argument);

#endif /* INC_NETWORK_PROCESSING_H_ */
