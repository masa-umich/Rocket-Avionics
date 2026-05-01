#ifndef LIMESTONE_INTERFACE_H
#define LIMESTONE_INTERFACE_H

#include "main.h"
#include <stdint.h>
#include "temp-header.h"
#include <stdio.h>
#include "apogee-detection-revised/ad-helpers.h"
#include "time.h"

typedef struct {
	float XL_x; // accel in x direction
	float XL_y; // y direction is axial to the rocket - positive points from engine to nose cone (upwards)
	float XL_z;
	float W_x; // angular velocity around x axis
	float W_y;
	float W_z;
} IMU_values;

uint32_t getTime_dbg(); // gets time in ms

void deployPilot_dbg(FILE *fptr_out);  // function to deploy pilot chute

void deployDrogue_dbg(FILE *fptr_out); // function to deploy drogue chute

void deployMain_dbg(FILE *fptr_out); // function to deploy main chute

void wait_dbg(uint32_t ms); // wait function in ms

void wait_until_dbg(uint32_t target_time_ms); // wait until target time in ms

void get_sensor_data_dbg(float* bar1, float* bar2,
					 IMU_values* imu1, IMU_values* imu2,
                     float* bar1_temp_C, float* bar2_temp_C, FILE *fptr_in); // function to get sensor data from IMU and barometers

int valves_open_dbg(); // returns 1 if both valves are open, 0 otherwise

uint32_t time_since_dbg();

//void low_power_mode_dbg();

#endif