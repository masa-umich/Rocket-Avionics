#ifndef AUTOSEQUENCE_SCRIPT_H
#define AUTOSEQUENCE_SCRIPT_H

#include "limestone-interface.h"
#include <stdlib.h>
#include <stdint.h>
#include "apogee-functions.h"
#include "main.h"

#define STATE0_NAME "Open MPV1"
#define STATE1_NAME "Open both MPVs"

#define STARTUP_MESSAGE "Opening Ox MPV"
#define STATE01_TRANSITION_MESSAGE "Opening Ox MPV"
//#define STATE01_TRANSITION_MESSAGE "Opening Ox MPV"   

const int MPV_DELAY = 0.5 * 1000; // in milliseconds
const int BURN_DURATION = 22 * 1000; // in milliseconds
const int MAX_MECO_WAIT = 30 * 1000; // millis
const uint32_t sampling_frequency = 20; // in Hz
//const uint32_t period = pdMS_TO_TICKS(1000) / sampling_frequency;
const uint32_t period = 1000.0f / sampling_frequency;

const uint32_t dt_read_vel = 100; // time between velocity readings in ms
const uint32_t dt_read_height = 10; // time between height readings in ms

void execute_flight_autosequence();


#endif