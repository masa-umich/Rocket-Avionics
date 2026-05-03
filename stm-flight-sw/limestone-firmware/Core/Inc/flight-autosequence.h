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
#include "system-state.h"
#include "eeprom-config.h"
#include "ad-functions.h"

extern EEPROM_conf_t loaded_config;

#define AUTOSEQUENCE_TASK_STACK		(uint32_t) 512 * 8

#define AUTOS_ARM_FLAG			(0x00000001U << 0)
#define AUTOS_ABORT_FLAG		(0x00000001U << 1)
#define AUTOS_OX_FLAG			(0x00000001U << 2)
#define AUTOS_FUEL_FLAG			(0x00000001U << 3)

//#define AUTOS_TEST

typedef struct {
	float XL_x; // accel in x direction
	float XL_y; // y direction is axial to the rocket - positive points from engine to nose cone (upwards)
	float XL_z;
	float W_x; // angular velocity around x axis
	float W_y;
	float W_z;
} IMU_values;

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
    uint32_t fluctus_apogee_timestamp;

    uint8_t override_calc_fallback_timers;

    uint8_t fluctus_disabled;
    uint8_t heights_recorded;

    float main_deploy_pressure;
    float drogue_deploy_pressure;

    float t_ground;
    float p_ground;
} Autos_boot_t;

void AutosequenceTask(void *argument);

int coldflow_autosequence(Autos_boot_t params);

void setup_autosequence();

void trigger_Fuel();

void trigger_Ox();

void trigger_arm();

void trigger_abort();

uint32_t getTime(); // gets time in ms

void deployPilot(); // function to deploy pilot chute

void deployDrogue(); // function to deploy drogue chute

void deployMain(); // function to deploy main chute

void wait(uint32_t ms); // wait function in ms

uint32_t time_since(uint32_t time_other);

void wait_until(uint32_t target_time_ms); // wait until target time in ms

void get_sensor_data(float* bar1, float* bar2,
					 IMU_values* imu1, IMU_values* imu2,
                     float* bar1_temp_C, float* bar2_temp_C); // function to get sensor data from IMU and barometers pressure in hPa, accel in m/s^2, angular velocity in deg/s

uint8_t valves_open(); // returns 1 if both valves are open, 0 otherwise

uint8_t should_abort(); // returns 1 if the autosequence should abort, 0 otherwise

void update_state_in_telem(FlightPhase status); // update state machine channel in telemetry

void update_boot_params(Autos_boot_t * params);

uint8_t get_fluctus_apogee();

uint8_t get_fluctus_5k();

uint8_t get_fluctus_1k();

#endif /* INC_FLIGHT_AUTOSEQUENCE_H_ */
