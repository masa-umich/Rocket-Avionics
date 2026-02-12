/*
 * flight-autosequence.h
 *
 *  Created on: Feb 12, 2026
 *      Author: felix
 */

#ifndef INC_FLIGHT_AUTOSEQUENCE_H_
#define INC_FLIGHT_AUTOSEQUENCE_H_

#include "main.h"
#include "log_errors.h"
#include "logging.h"

#define AUTOSEQUENCE_TASK_STACK		(uint32_t) 512 * 4
#define ARM_TIMEOUT					15 * 1000 // ticks or ms

void AutosequenceTask(void *argument);

void setup_autosequence();

void trigger_Fuel();

void trigger_Ox();

#endif /* INC_FLIGHT_AUTOSEQUENCE_H_ */
