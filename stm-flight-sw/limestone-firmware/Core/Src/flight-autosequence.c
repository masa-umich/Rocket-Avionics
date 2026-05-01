/*
 * flight-autosequence.c
 *
 *  Created on: Feb 12, 2026
 *      Author: felix
 */

#include "flight-autosequence.h"
#include "autosequence-script.h"

osEventFlagsId_t autos_events;

void setup_autosequence() {
	autos_events = osEventFlagsNew(NULL);
}

void AutosequenceTask(void *argument) {
	Autos_boot_t * boot_params = (Autos_boot_t *) argument;
	for(;;) {
		osEventFlagsClear(autos_events, AUTOS_ARM_FLAG | AUTOS_ABORT_FLAG | AUTOS_OX_FLAG | AUTOS_FUEL_FLAG);
		uint32_t flags = boot_params->phase > AUTOS_STATE_DEARMED ? (boot_params->phase < AUTOS_STATE_DONE ? AUTOS_ARM_FLAG : 0) : osEventFlagsWait(autos_events, AUTOS_ARM_FLAG, osFlagsWaitAny, osWaitForever);
		if(!(flags & osFlagsError) && (flags & AUTOS_ARM_FLAG)) {
			osEventFlagsClear(autos_events, AUTOS_ARM_FLAG | AUTOS_ABORT_FLAG | AUTOS_OX_FLAG | AUTOS_FUEL_FLAG);
			log_message(FC_STAT_ARMED, -1);
			update_state_in_telem(boot_params->phase > AUTOS_STATE_DEARMED ? boot_params->phase : AUTOS_STATE_ARMED);
			if(boot_params->phase == AUTOS_STATE_DEARMED) {
				Autos_boot_t arm_state = {0};
				arm_state.phase = AUTOS_STATE_ARMED;
				update_boot_params(&arm_state);
			}
#ifndef AUTOS_TEST
			int exit_stat = execute_flight_autosequence(*boot_params);
#else
			int exit_stat = coldflow_autosequence(*boot_params);
#endif

			if(exit_stat != 0) {
				update_state_in_telem(AUTOS_STATE_DEARMED);
				Autos_boot_t end_state = {0};
				end_state.phase = AUTOS_STATE_DEARMED;
				update_boot_params(&end_state);
				osDelay(1);
				boot_params->phase = AUTOS_STATE_DEARMED;
				continue;
			}
		}
		else if(flags & osFlagsError) {
			osDelay(100);
			continue;
		}
		update_state_in_telem(AUTOS_STATE_DONE);
		Autos_boot_t done_state = {0};
		done_state.phase = AUTOS_STATE_DONE;
		update_boot_params(&done_state);
		for(;;) {
			uint32_t end_flags = osEventFlagsWait(autos_events, AUTOS_ABORT_FLAG, osFlagsWaitAny, osWaitForever);
			if(!(end_flags & osFlagsError) && (end_flags & AUTOS_ABORT_FLAG)) {
				break;
			}
			osDelay(100);
		}
		update_state_in_telem(AUTOS_STATE_DEARMED);
		Autos_boot_t end_state = {0};
		end_state.phase = AUTOS_STATE_DEARMED;
		update_boot_params(&end_state);
		osDelay(1);
		boot_params->phase = AUTOS_STATE_DEARMED;
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

uint32_t time_since(uint32_t time_other) {
	return getTime() - time_other;
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
	if(xSemaphoreTake(Rocket_h.fcState_access, 5) == pdPASS) {
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

int coldflow_autosequence(Autos_boot_t params) {
	uint8_t entry_state = params.phase;
	uint32_t start = getTime();
	while(getTime() - start < 20000 && entry_state < AUTOS_STATE_BURN) {
		if(valves_open()) {
			break;
		}
		if(should_abort()) {
			return -1;
		}
		wait(100);
	}
	if(getTime() - start >= 20000) return -1;
	for(uint8_t i = entry_state > AUTOS_STATE_BURN ? entry_state : AUTOS_STATE_BURN;i < AUTOS_STATE_DONE;i++) {
		update_state_in_telem(i);
		Autos_boot_t cur_state = {0};
		cur_state.phase = i;
		update_boot_params(&cur_state);
		start = getTime();
		while(getTime() - start < 5000) {
			if(should_abort()) {
				return -1;
			}
			wait(100);
		}
	}
	return 0;
}

void update_boot_params(Autos_boot_t * params) {
	write_autosequence_params((void *)params, sizeof(Autos_boot_t));
}

uint8_t get_fluctus_apogee() {
	if(xSemaphoreTake(Rocket_h.fcState_access, 5) == pdPASS) {
		uint8_t state = Rocket_h.fcState.fluctus_apogee_state;
		xSemaphoreGive(Rocket_h.fcState_access);
		return state;
	}
	return 0;
}

uint8_t get_fluctus_5k() {
	if(xSemaphoreTake(Rocket_h.fcState_access, 5) == pdPASS) {
		uint8_t state = Rocket_h.fcState.fluctus_5k_state;
		xSemaphoreGive(Rocket_h.fcState_access);
		return state;
	}
	return 0;
}

uint8_t get_fluctus_1k() {
	if(xSemaphoreTake(Rocket_h.fcState_access, 5) == pdPASS) {
		uint8_t state = Rocket_h.fcState.fluctus_1k_state;
		xSemaphoreGive(Rocket_h.fcState_access);
		return state;
	}
	return 0;
}

void log_autos_info(Autos_boot_t * params) {
	char logmsg[sizeof(STAT_AVAILABLE_FLASH) + 37];
	snprintf(logmsg, sizeof(logmsg), STAT_AVAILABLE_FLASH "%" PRIu32 "B: %" PRIu32 "KB/512MB %u%%", available, available >> 10, percent);
}
