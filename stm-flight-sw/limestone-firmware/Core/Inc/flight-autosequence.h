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
#include "autosequence-script.h"

#define AUTOSEQUENCE_TASK_STACK		(uint32_t) 512 * 4

#define AUTOS_ARM_FLAG			(0x00000001U << 0)
#define AUTOS_ABORT_FLAG		(0x00000001U << 1)
#define AUTOS_OX_FLAG			(0x00000001U << 2)
#define AUTOS_FUEL_FLAG			(0x00000001U << 3)

#define AUTOS_TEST

typedef struct {
	float XL_x; // accel in x direction
	float XL_y; // y direction is axial to the rocket - positive points from engine to nose cone (upwards)
	float XL_z;
	float W_x; // angular velocity around x axis
	float W_y;
	float W_z;
} IMU_values;

typedef enum {
	AUTOS_STATE_DEARMED = 0x00,
	AUTOS_STATE_ARMED = 0x01
} AutoS_SM; // TODO ADD TO THIS

void AutosequenceTask(void *argument);

void setup_autosequence();

void trigger_Fuel();

void trigger_Ox();

void trigger_arm();

void trigger_abort();

uint32_t getTime(); // gets time in ms

void deployPilot(); // function to deploy pilot chute // TODO implement

void deployDrogue(); // function to deploy drogue chute // TODO implement

void deployMain(); // function to deploy main chute // TODO implement

void wait(uint32_t ms); // wait function in ms

void wait_until(uint32_t target_time_ms); // wait until target time in ms

void get_sensor_data(float* bar1, float* bar2,
					 IMU_values* imu1, IMU_values* imu2,
                     float* bar1_temp_C, float* bar2_temp_C); // function to get sensor data from IMU and barometers pressure in hPa, accel in m/s^2, angular velocity in deg/s

uint8_t valves_open(); // returns 1 if both valves are open, 0 otherwise

uint8_t should_abort(); // returns 1 if the autosequence should abort, 0 otherwise

void update_state_in_telem(AutoS_SM status); // update state machine channel in telemetry

#endif /* INC_FLIGHT_AUTOSEQUENCE_H_ */
