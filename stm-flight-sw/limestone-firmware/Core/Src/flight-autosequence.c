/*
 * flight-autosequence.c
 *
 *  Created on: Feb 12, 2026
 *      Author: felix
 */

#include "flight-autosequence.h"

osEventFlagsId_t autos_events;

void setup_autosequence() {
	autos_events = osEventFlagsNew(NULL);
}

void AutosequenceTask(void *argument) {
	for(;;) {
		osEventFlagsClear(autos_events, AUTOS_ARM_FLAG | AUTOS_ABORT_FLAG | AUTOS_OX_FLAG | AUTOS_FUEL_FLAG);
		uint32_t flags = osEventFlagsWait(autos_events, AUTOS_ARM_FLAG, osFlagsWaitAny, osWaitForever);
		if(!(flags & osFlagsError) && (flags & AUTOS_ARM_FLAG)) {
			osEventFlagsClear(autos_events, AUTOS_ARM_FLAG | AUTOS_ABORT_FLAG | AUTOS_OX_FLAG | AUTOS_FUEL_FLAG);
			log_message(FC_STAT_ARMED, -1);
			update_state_in_telem(AUTOS_STATE_ARMED);
#ifndef AUTOS_TEST
			execute_flight_autosequence();
#else
			coldflow_autosequence();
#endif
		}
		update_state_in_telem(AUTOS_STATE_DEARMED);
		log_message(FC_STAT_DEARMED, -1);
		osDelay(1);
	}
}

void trigger_Ox() {
	osEventFlagsSet(autos_events, AUTOS_OX_FLAG);
}

void trigger_Fuel() {
	osEventFlagsSet(autos_events, AUTOS_FUEL_FLAG);
}

void trigger_arm() {
	osEventFlagsSet(autos_events, AUTOS_ARM_FLAG);
}

void trigger_abort() {
	osEventFlagsSet(autos_events, AUTOS_ABORT_FLAG);
}

uint32_t getTime() {
	return osKernelGetTickCount();
}

void wait(uint32_t ms) {
	osDelay(pdMS_TO_TICKS(ms));
}

void wait_until(uint32_t target_time_ms) {
	osDelayUntil(pdMS_TO_TICKS(target_time_ms));
}

void get_sensor_data(float* bar1, float* bar2,
					 IMU_values* imu1, IMU_values* imu2,
                     float* bar1_temp_C, float* bar2_temp_C) {
	if(xSemaphoreTake(Rocket_h.fcState_access, portMAX_DELAY) == pdPASS) {
		*bar1 = Rocket_h.fcState.bar1;
		*bar2 = Rocket_h.fcState.bar2;

		imu1->XL_x = Rocket_h.fcState.imu1_A.XL_x;
		imu1->XL_y = Rocket_h.fcState.imu1_A.XL_y;
		imu1->XL_z = Rocket_h.fcState.imu1_A.XL_z;
		imu1->W_x = Rocket_h.fcState.imu1_W.G_x;
		imu1->W_y = Rocket_h.fcState.imu1_W.G_y;
		imu1->W_z = Rocket_h.fcState.imu1_W.G_z;

		imu2->XL_x = Rocket_h.fcState.imu2_A.XL_x;
		imu2->XL_y = Rocket_h.fcState.imu2_A.XL_y;
		imu2->XL_z = Rocket_h.fcState.imu2_A.XL_z;
		imu2->W_x = Rocket_h.fcState.imu2_W.G_x;
		imu2->W_y = Rocket_h.fcState.imu2_W.G_y;
		imu2->W_z = Rocket_h.fcState.imu2_W.G_z;

		*bar1_temp_C = Rocket_h.fcState.bar1_temp;
		*bar2_temp_C = Rocket_h.fcState.bar2_temp;

		xSemaphoreGive(Rocket_h.fcState_access);
	}
}

uint8_t valves_open() {
	uint32_t flags = osEventFlagsGet(autos_events);
	uint32_t MPV_FLAGS = AUTOS_OX_FLAG | AUTOS_FUEL_FLAG;
	return ((flags & MPV_FLAGS) == MPV_FLAGS);
}

uint8_t should_abort() {
	uint32_t flags = osEventFlagsGet(autos_events);
	return ((flags & AUTOS_ABORT_FLAG) == AUTOS_ABORT_FLAG);
}

void update_state_in_telem(AutoS_SM status) {
	if(xSemaphoreTake(Rocket_h.fcState_access, portMAX_DELAY) == pdPASS) {
		Rocket_h.fcState.auto_sequence_state = status;
		xSemaphoreGive(Rocket_h.fcState_access);
	}
}

void deployPilot() {
	set_valve_within((Valve_Channel) loaded_config.pilot_para_index, Valve_Energized);
}

void deployDrogue() {
	set_valve_within((Valve_Channel) loaded_config.drogue_para_index, Valve_Energized);
}

void deployMain() {
	set_valve_within((Valve_Channel) loaded_config.main_para_index, Valve_Energized);
}

void coldflow_autosequence() {
	uint32_t start = getTime();
	while(getTime() - start < 30000) {
		if(valves_open()) {
			break;
		}
		if(should_abort()) {
			return;
		}
		wait(100);
	}
	if(getTime() - start >= 30000) return;
	wait(1000);
	deployPilot();
	wait(500);
}
