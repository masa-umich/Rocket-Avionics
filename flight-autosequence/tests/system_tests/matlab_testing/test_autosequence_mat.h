#ifndef AUTOSEQUENCE_SCRIPT_H
#define AUTOSEQUENCE_SCRIPT_H

//#include "main.h"
//#include "flight-autosequence.h"
#include <stdlib.h>
#include <stdint.h>
#include "ad-functions.h"
#include "sim_hardware.h"

#define AUTOSEQUENCE_DEBUG 0

#define ALTITUDE_BUFFER_SIZE 31
#define ALT_BUF_ELT_SPACING 3 // record every 3rd altitude reading to ensure we have a good spread of data points for curve fitting

// CONSTANT TIMER DEFINITIONS - MODIFY BEFORE LAUNCH
extern const uint32_t APOGEE_CONSTANT_TIMER;            // 60 seconds in milliseconds
extern const uint32_t DROGUE_CONSTANT_TIMER;            // 1200 seconds in milliseconds
extern const uint32_t MAIN_CONSTANT_TIMER;              // 1500 seconds in milliseconds


// SAMPLING PERIOD
extern const uint32_t period;                           // ms, sampling period


// TIMEOUT DEFINITIONS 
extern const uint32_t MAX_HANDOFF_TO_VALVE_OPEN_MS;     // 15 seconds
extern const uint32_t LOCKOUT_END_TIME;                 // 30 seconds, hard cutoff for lockout phase end
extern const uint32_t APOGEE_AGREEMENT_WINDOW;          // 2 seconds

typedef struct {
    FlightPhase phase;
    uint32_t current_time_in_flight;

    uint32_t fallback_apogee_time;
    uint32_t fallback_5k_time;
    uint32_t fallback_1k_time;

    uint8_t apogee_detection_worked;
    uint8_t fallback_timers_worked;
    uint8_t constant_timers_worked;

    uint8_t fluctus_apogee_detected;
    uint8_t fluctus_5k_detected;
    uint8_t fluctus_1k_detected;
    
    uint32_t fluctus_apogee_timestamp;
    uint32_t fluctus_5k_timestamp;
    uint32_t fluctus_1k_timestamp;

    uint8_t override_calc_fallback_timers;

    uint8_t fluctus_disabled;
    uint8_t heights_recorded;

    float main_deploy_pressure;
    float drogue_deploy_pressure;

    float t_ground;
    float p_ground;
} Autos_boot_t;

int execute_flight_autosequence(Autos_boot_t boot_params);

#endif
