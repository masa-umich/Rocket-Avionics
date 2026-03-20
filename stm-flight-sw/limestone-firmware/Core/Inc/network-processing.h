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
#include "flight-autosequence.h"

void ProcessPackets(void *argument);

void set_valve_within(Valve_Channel valve, Valve_State_t desiredState);

#endif /* INC_NETWORK_PROCESSING_H_ */
