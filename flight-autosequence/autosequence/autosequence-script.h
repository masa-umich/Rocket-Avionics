#ifndef AUTOSEQUENCE_SCRIPT_H
#define AUTOSEQUENCE_SCRIPT_H

#include "limestone-interface.h"
#include <stdlib.h>
#include <stdint.h>
#include "main.h"   

// CONSTANT TIMER DEFINITIONS - MODIFY AS NEEDED
const uint32_t APOGEE_CONSTANT_TIMER = 100 * 1000; // 100 seconds in milliseconds
const uint32_t DROGUE_CONSTANT_TIMER = 120 * 1000; // 120 seconds in milliseconds
const uint32_t MAIN_CONSTANT_TIMER = 150 * 1000; // 150 seconds in milliseconds


// System defs
const uint32_t sampling_frequency = 20; // in Hz
//const uint32_t period = pdMS_TO_TICKS(1000) / sampling_frequency;
const uint32_t period = 1000.0f / sampling_frequency;


const int ALTITUDE_BUFFER_SIZE = 11;
const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS = 15000; // 15 seconds


void execute_flight_autosequence();


#endif