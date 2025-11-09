/*
 * board-state.h
 *
 *  Created on: Nov 9, 2025
 *      Author: felix
 */

#ifndef INC_BOARD_STATE_H_
#define INC_BOARD_STATE_H_

#include "main.h"
#include "VLVs.h"
#include "LSM6DSO32XTR.h"
#include "messages.h"
#include "log_errors.h"

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
	SemaphoreHandle_t bbState_access;
	SemaphoreHandle_t bbValve_access;
	Bay_Board_State_t bbState;
	Valve bbValves[5];
	Valve_State_t bbValveStates[7];
} Board_State_t;

extern Board_State_t Board_h;

void setup_system_state();

void setup_valve(uint8_t index, GPIO_TypeDef * EN_Port, uint16_t EN_Pin, GPIO_TypeDef * OLD_Port, uint16_t OLD_Pin);

int pack_bb_telemetry_msg(TelemetryMessage *msg, uint64_t timestamp, uint8_t timeout_ticks);

Valve_State_t set_and_update_valve(Valve_Channel valve, Valve_State_t desiredState);

#endif /* INC_BOARD_STATE_H_ */
