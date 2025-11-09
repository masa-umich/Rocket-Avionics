/*
 * board-state.c
 *
 *  Created on: Nov 9, 2025
 *      Author: felix
 */

#include "board-state.h"

Board_State_t Board_h = {0};

void setup_system_state() {
	// Init rocket state struct
	memset(&Board_h, 0, sizeof(Board_h)); // Reset contents
	Board_h.bbState_access = xSemaphoreCreateMutex();
	Board_h.bbValve_access = xSemaphoreCreateMutex();
}

void setup_valve(uint8_t index, GPIO_TypeDef * EN_Port, uint16_t EN_Pin, GPIO_TypeDef * OLD_Port, uint16_t OLD_Pin) {
	Board_h.bbValves[index].VLV_EN_GPIO_Port = EN_Port;
	Board_h.bbValves[index].VLV_EN_GPIO_Pin = EN_Pin;
	Board_h.bbValves[index].VLV_OLD_GPIO_Port = OLD_Port;
	Board_h.bbValves[index].VLV_OLD_GPIO_Pin = OLD_Pin;
	Board_h.bbValveStates[index] = VLV_State(Board_h.bbValves[index]);
}

// Pack telemetry message
// returns 0 if successful, 1 if the telemetry data could not be accessed
// timeout_ticks: how many FreeRTOS ticks to wait before giving up
int pack_bb_telemetry_msg(TelemetryMessage *msg, uint64_t timestamp, uint8_t timeout_ticks) {
	msg->board_id = bb_num;
	msg->num_channels = BB1_TELEMETRY_CHANNELS; // all the bay boards have the same number of channels
	msg->timestamp = timestamp;
	if(xSemaphoreTake(Board_h.bbState_access, timeout_ticks) == pdPASS) {
		msg->telemetry_data[BB1_VLV1_CURRENT_I] 	= Board_h.bbState.vlv1_current;
		msg->telemetry_data[BB1_VLV1_OLD_I] 		= Board_h.bbState.vlv1_old;
		msg->telemetry_data[BB1_VLV2_CURRENT_I] 	= Board_h.bbState.vlv2_current;
		msg->telemetry_data[BB1_VLV2_OLD_I] 		= Board_h.bbState.vlv2_old;
		msg->telemetry_data[BB1_VLV3_CURRENT_I] 	= Board_h.bbState.vlv3_current;
		msg->telemetry_data[BB1_VLV3_OLD_I] 		= Board_h.bbState.vlv3_old;
		msg->telemetry_data[BB1_VLV4_CURRENT_I] 	= Board_h.bbState.vlv4_current;
		msg->telemetry_data[BB1_VLV4_OLD_I] 		= Board_h.bbState.vlv4_old;
		msg->telemetry_data[BB1_VLV5_CURRENT_I] 	= Board_h.bbState.vlv5_current;
		msg->telemetry_data[BB1_VLV5_OLD_I] 		= Board_h.bbState.vlv5_old;
		msg->telemetry_data[BB1_VLV6_CURRENT_I] 	= Board_h.bbState.vlv6_current;
		msg->telemetry_data[BB1_VLV6_OLD_I] 		= Board_h.bbState.vlv6_old;
		msg->telemetry_data[BB1_VLV7_CURRENT_I] 	= Board_h.bbState.vlv7_current;
		msg->telemetry_data[BB1_VLV7_OLD_I] 		= Board_h.bbState.vlv7_old;
		msg->telemetry_data[BB1_PT_1_I] 			= Board_h.bbState.pt1;
		msg->telemetry_data[BB1_PT_2_I] 			= Board_h.bbState.pt2;
		msg->telemetry_data[BB1_PT_3_I] 			= Board_h.bbState.pt3;
		msg->telemetry_data[BB1_PT_4_I] 			= Board_h.bbState.pt4;
		msg->telemetry_data[BB1_PT_5_I] 			= Board_h.bbState.pt5;
		msg->telemetry_data[BB1_PT_6_I] 			= Board_h.bbState.pt6;
		msg->telemetry_data[BB1_PT_7_I] 			= Board_h.bbState.pt7;
		msg->telemetry_data[BB1_PT_8_I] 			= Board_h.bbState.pt8;
		msg->telemetry_data[BB1_PT_9_I] 			= Board_h.bbState.pt9;
		msg->telemetry_data[BB1_PT_10_I] 			= Board_h.bbState.pt10;
		msg->telemetry_data[BB1_TC_1_I] 			= Board_h.bbState.tc1;
		msg->telemetry_data[BB1_TC_2_I] 			= Board_h.bbState.tc2;
		msg->telemetry_data[BB1_TC_3_I] 			= Board_h.bbState.tc3;
		msg->telemetry_data[BB1_TC_4_I] 			= Board_h.bbState.tc4;
		msg->telemetry_data[BB1_TC_5_I] 			= Board_h.bbState.tc5;
		msg->telemetry_data[BB1_TC_6_I] 			= Board_h.bbState.tc6;
		msg->telemetry_data[BB1_IMU1_X_I] 			= Board_h.bbState.imu1_A.XL_x;
		msg->telemetry_data[BB1_IMU1_Y_I] 			= Board_h.bbState.imu1_A.XL_y;
		msg->telemetry_data[BB1_IMU1_Z_I] 			= Board_h.bbState.imu1_A.XL_z;
		msg->telemetry_data[BB1_IMU1_WP_I] 			= Board_h.bbState.imu1_W.G_y;
		msg->telemetry_data[BB1_IMU1_WR_I] 			= Board_h.bbState.imu1_W.G_x;
		msg->telemetry_data[BB1_IMU1_WY_I] 			= Board_h.bbState.imu1_W.G_z;
		msg->telemetry_data[BB1_IMU2_X_I] 			= Board_h.bbState.imu2_A.XL_x;
		msg->telemetry_data[BB1_IMU2_Y_I] 			= Board_h.bbState.imu2_A.XL_y;
		msg->telemetry_data[BB1_IMU2_Z_I] 			= Board_h.bbState.imu2_A.XL_z;
		msg->telemetry_data[BB1_IMU2_WP_I] 			= Board_h.bbState.imu2_W.G_y;
		msg->telemetry_data[BB1_IMU2_WR_I] 			= Board_h.bbState.imu2_W.G_x;
		msg->telemetry_data[BB1_IMU2_WY_I] 			= Board_h.bbState.imu2_W.G_z;
		msg->telemetry_data[BB1_BAR_1_I] 			= Board_h.bbState.bar1;
		msg->telemetry_data[BB1_BAR_2_I] 			= Board_h.bbState.bar2;
		msg->telemetry_data[BB1_24V_VOLTAGE_I] 		= Board_h.bbState.bus24v_voltage;
		msg->telemetry_data[BB1_24V_CURRENT_I] 		= Board_h.bbState.bus24v_current;
		msg->telemetry_data[BB1_12V_VOLTAGE_I] 		= Board_h.bbState.bus12v_voltage;
		msg->telemetry_data[BB1_12V_CURRENT_I] 		= Board_h.bbState.bus12v_current;
		msg->telemetry_data[BB1_5V_VOLTAGE_I] 		= Board_h.bbState.bus5v_voltage;
		msg->telemetry_data[BB1_5V_CURRENT_I] 		= Board_h.bbState.bus5v_current;
		msg->telemetry_data[BB1_3V3_VOLTAGE_I] 		= Board_h.bbState.bus3v3_voltage;
		msg->telemetry_data[BB1_3V3_CURRENT_I] 		= Board_h.bbState.bus3v3_current;
		xSemaphoreGive(Board_h.bbState_access);
	}
	else {
		return 1;
	}

	return 0;
}

// Set a valve to a state and update its corresponding valve state
// Returns the state of the valve after setting it (this should be equal to desiredState, but in the case of an error, it will accurately reflect the current state of the valve)
Valve_State_t set_and_update_valve(Valve_Channel valve, Valve_State_t desiredState) {
	if(valve > Vlv5) {
		return Valve_Deenergized;
	}
	uint8_t vlverror = 0;
	 if(xSemaphoreTake(Board_h.bbValve_access, 5) == pdPASS) {
		 if(desiredState == Valve_Energized) {
			 VLV_En(Board_h.bbValves[valve]);
			 Board_h.bbValveStates[valve] = Valve_Energized;
		 }
		 else if(desiredState == Valve_Deenergized) {
			 VLV_Den(Board_h.bbValves[valve]);
			 Board_h.bbValveStates[valve] = Valve_Deenergized;
		 }
		 else {
			 // Error invalid state
			 vlverror = 1;
		 }
		 xSemaphoreGive(Board_h.bbValve_access);
	 }
	 else {
		 vlverror = 1;
	 }
	 if(!vlverror) {
		 return desiredState;
	 }
	 else {
		 return HAL_GPIO_ReadPin(Board_h.bbValves[valve].VLV_EN_GPIO_Port, Board_h.bbValves[valve].VLV_EN_GPIO_Pin);
	 }
}
