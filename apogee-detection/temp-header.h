#include "FreeRTOS.h"
#include "task.h"
#include "LSM6DSO32XTR.h"
#include "VLVs.h"
#include "semphr.h"

typedef struct {
	float pt1;
	float pt2;
	float pt3;
	float pt4;
	float pt5;
	float pt6;
	float pt7;
	float pt8;
	float pt9;
	float pt10;
	float tc1;
	float tc2;
	float tc3;
	float tc4;
	float tc5;
	float tc6;
	Accel imu1_A;
	Accel imu2_A;
	AngRate imu1_W;
	AngRate imu2_W;
	float bar1;
	float bar2;
	float bus24v_voltage;
	float bus24v_current;
	float bus12v_voltage;
	float bus12v_current;
	float bus5v_voltage;
	float bus5v_current;
	float bus3v3_voltage;
	float bus3v3_current;
	float vlv1_current;
	VLV_OpenLoad vlv1_old;
	float vlv2_current;
	VLV_OpenLoad vlv2_old;
	float vlv3_current;
	VLV_OpenLoad vlv3_old;
	float vlv4_current;
	VLV_OpenLoad vlv4_old;
	float vlv5_current;
	VLV_OpenLoad vlv5_old;
	float vlv6_current;
	VLV_OpenLoad vlv6_old;
	float vlv7_current;
	VLV_OpenLoad vlv7_old;

	uint64_t timestamp;
} Bay_Board_State_t;

typedef struct {
	float pt1;
	float pt2;
	float pt3;
	float pt4;
	float pt5;
	float tc1;
	float tc2;
	float tc3;
	Accel imu1_A;
	Accel imu2_A;
	AngRate imu1_W;
	AngRate imu2_W;
	float gps_lat;
	float gps_long;
	float gps_alt;
	float bar1;
	float bar2;
	float bus24v_voltage;
	float bus24v_current;
	float bus12v_voltage;
	float bus12v_current;
	float bus5v_voltage;
	float bus5v_current;
	float bus3v3_voltage;
	float bus3v3_current;
	float vlv1_current;
	VLV_OpenLoad vlv1_old;
	float vlv2_current;
	VLV_OpenLoad vlv2_old;
	float vlv3_current;
	VLV_OpenLoad vlv3_old;

	uint64_t timestamp; // Only used to monitor how "fresh" the data is
} Flight_Computer_State_t;

typedef struct {
	Accel imu1_A;
	Accel imu2_A;
	AngRate imu1_W;
	AngRate imu2_W;
	float bar1;
	float bar2;

	uint64_t timestamp;
} Flight_Recorder_State_t;

typedef struct {
	SemaphoreHandle_t fcState_access;
	SemaphoreHandle_t fcValve_access;
	Flight_Computer_State_t fcState;
	Valve fcValves[3];
	Valve_State_t fcValveStates[3];

	SemaphoreHandle_t bb1State_access;
	SemaphoreHandle_t bb1Valve_access;
	Bay_Board_State_t bb1State;
	Valve_State_t bb1ValveStates[7];

	SemaphoreHandle_t bb2State_access;
	SemaphoreHandle_t bb2Valve_access;
	Bay_Board_State_t bb2State;
	Valve_State_t bb2ValveStates[7];

	SemaphoreHandle_t bb3State_access;
	SemaphoreHandle_t bb3Valve_access;
	Bay_Board_State_t bb3State;
	Valve_State_t bb3ValveStates[7];

	SemaphoreHandle_t frState_access;
	Flight_Recorder_State_t frState;
} Rocket_State_t;

extern Rocket_State_t Rocket_h; //main rocket handle 