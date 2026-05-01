#ifndef AUTOSEQUENCE_SCRIPT_H
#define AUTOSEQUENCE_SCRIPT_H

#include "main.h"
#include "flight-autosequence.h"
#include <stdlib.h>
#include <stdint.h>
#include "ad-helpers.h"

#define AUTOSEQUENCE_DEBUG 0

#define ALTITUDE_BUFFER_SIZE 31
#define ALT_BUF_ELT_SPACING 3 // record every 3rd altitude reading to ensure we have a good spread of data points for curve fitting

// CONSTANT TIMER DEFINITIONS - MODIFY AS NEEDED
extern const uint32_t APOGEE_CONSTANT_TIMER; // 60 seconds in milliseconds
extern const uint32_t DROGUE_CONSTANT_TIMER; // 1200 seconds in milliseconds
extern const uint32_t MAIN_CONSTANT_TIMER; // 1500 seconds in milliseconds


// System defs
extern const uint32_t period; // ms, sampling period

extern const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS; // 15 seconds
extern const uint32_t LOCKOUT_END_TIME; // 30 seconds, hard cutoff for lockout phase end
extern const uint32_t APOGEE_AGREEMENT_WINDOW; // 2 seconds

int execute_flight_autosequence(Autos_boot_t * boot_params);

#endif
