#ifndef AUTOSEQUENCE_SCRIPT_H
#define AUTOSEQUENCE_SCRIPT_H

#include "main.h"
#include "limestone-interface.h"
#include <stdlib.h>
#include <stdint.h>
#include "ad-helpers.h"

#define AUTOSEQUENCE_DEBUG 0

// CONSTANT TIMER DEFINITIONS - MODIFY AS NEEDED
extern const uint32_t APOGEE_CONSTANT_TIMER; // 100 seconds in milliseconds
extern const uint32_t DROGUE_CONSTANT_TIMER; // 120 seconds in milliseconds
extern const uint32_t MAIN_CONSTANT_TIMER; // 150 seconds in milliseconds


// System defs
extern const uint32_t period; // ms, sampling period

extern const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS; // 15 seconds
extern const uint32_t MAX_BURN_DURATION_MS; // 22 seconds

extern const uint32_t MIN_LOCKOUT_WAIT_TIME; // ms, minimum time we expect to wait in lockout phase
extern const uint32_t MAX_LOCKOUT_WAIT_TIME; // ms, maximum time we expect to wait in lockout phase
extern const int WAIT_TIME_MULTIPLIER;

void execute_flight_autosequence();


#endif
