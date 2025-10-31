/*
 * system-state.c
 *
 *  Created on: Oct 21, 2025
 *      Author: felix
 */


#include "system-state.h"

Rocket_State_t Rocket_h = {0};

void setup_system_state() {
	// Init rocket state struct
	memset(&Rocket_h, 0, sizeof(Rocket_h)); // Reset contents
	Rocket_h.fcState_access = xSemaphoreCreateMutex();
	Rocket_h.fcValve_access = xSemaphoreCreateMutex();
	Rocket_h.bb1State_access = xSemaphoreCreateMutex();
	Rocket_h.bb1Valve_access = xSemaphoreCreateMutex();
	Rocket_h.bb2State_access = xSemaphoreCreateMutex();
	Rocket_h.bb2Valve_access = xSemaphoreCreateMutex();
	Rocket_h.bb3State_access = xSemaphoreCreateMutex();
	Rocket_h.bb3Valve_access = xSemaphoreCreateMutex();
}

void setup_valve(uint8_t index, GPIO_TypeDef * EN_Port, uint16_t EN_Pin, GPIO_TypeDef * OLD_Port, uint16_t OLD_Pin) {
  	Rocket_h.fcValves[index].VLV_EN_GPIO_Port = EN_Port;
  	Rocket_h.fcValves[index].VLV_EN_GPIO_Pin = EN_Pin;
  	Rocket_h.fcValves[index].VLV_OLD_GPIO_Port = OLD_Port;
  	Rocket_h.fcValves[index].VLV_OLD_GPIO_Pin = OLD_Pin;
  	Rocket_h.fcValveStates[index] = VLV_State(Rocket_h.fcValves[index]);
}

// Pack telemetry message
// returns 0 if successful, 1 if the telemetry data could not be accessed
// timeout_ticks: how many FreeRTOS ticks to wait before giving up
int pack_fc_telemetry_msg(TelemetryMessage *msg, uint64_t timestamp, uint8_t timeout_ticks) {
	msg->board_id = BOARD_FC;
	msg->num_channels = FC_TELEMETRY_CHANNELS;
	msg->timestamp = timestamp;
	if(xSemaphoreTake(Rocket_h.fcState_access, timeout_ticks) == pdPASS) {
		msg->telemetry_data[FC_VLV1_CURRENT_I] = Rocket_h.fcState.vlv1_current;
		msg->telemetry_data[FC_VLV1_OLD_I] = Rocket_h.fcState.vlv1_old;
		msg->telemetry_data[FC_VLV2_CURRENT_I] = Rocket_h.fcState.vlv2_current;
		msg->telemetry_data[FC_VLV2_OLD_I] = Rocket_h.fcState.vlv2_old;
		msg->telemetry_data[FC_VLV3_CURRENT_I] = Rocket_h.fcState.vlv3_current;
		msg->telemetry_data[FC_VLV3_OLD_I] = Rocket_h.fcState.vlv3_old;
		msg->telemetry_data[FC_PT_1_I] = Rocket_h.fcState.pt1;
		msg->telemetry_data[FC_PT_2_I] = Rocket_h.fcState.pt2;
		msg->telemetry_data[FC_PT_3_I] = Rocket_h.fcState.pt3;
		msg->telemetry_data[FC_PT_4_I] = Rocket_h.fcState.pt4;
		msg->telemetry_data[FC_PT_5_I] = Rocket_h.fcState.pt5;
		msg->telemetry_data[FC_TC_1_I] = Rocket_h.fcState.tc1;
		msg->telemetry_data[FC_TC_2_I] = Rocket_h.fcState.tc2;
		msg->telemetry_data[FC_TC_3_I] = Rocket_h.fcState.tc3;
		msg->telemetry_data[FC_IMU1_X_I] = Rocket_h.fcState.imu1_A.XL_x;
		msg->telemetry_data[FC_IMU1_Y_I] = Rocket_h.fcState.imu1_A.XL_y;
		msg->telemetry_data[FC_IMU1_Z_I] = Rocket_h.fcState.imu1_A.XL_z;
		msg->telemetry_data[FC_IMU1_WP_I] = Rocket_h.fcState.imu1_W.G_y;
		msg->telemetry_data[FC_IMU1_WR_I] = Rocket_h.fcState.imu1_W.G_x;
		msg->telemetry_data[FC_IMU1_WY_I] = Rocket_h.fcState.imu1_W.G_z;
		msg->telemetry_data[FC_IMU2_X_I] = Rocket_h.fcState.imu2_A.XL_x;
		msg->telemetry_data[FC_IMU2_Y_I] = Rocket_h.fcState.imu2_A.XL_y;
		msg->telemetry_data[FC_IMU2_Z_I] = Rocket_h.fcState.imu2_A.XL_z;
		msg->telemetry_data[FC_IMU2_WP_I] = Rocket_h.fcState.imu2_W.G_y;
		msg->telemetry_data[FC_IMU2_WR_I] = Rocket_h.fcState.imu2_W.G_x;
		msg->telemetry_data[FC_IMU2_WY_I] = Rocket_h.fcState.imu2_W.G_z;
		msg->telemetry_data[FC_GPS_LAT_I] = Rocket_h.fcState.gps_lat;
		msg->telemetry_data[FC_GPS_LONG_I] = Rocket_h.fcState.gps_long;
		msg->telemetry_data[FC_GPS_ALT_I] = Rocket_h.fcState.gps_alt;
		msg->telemetry_data[FC_BAR_1_I] = Rocket_h.fcState.bar1;
		msg->telemetry_data[FC_BAR_2_I] = Rocket_h.fcState.bar2;
		msg->telemetry_data[FC_24V_VOLTAGE_I] = Rocket_h.fcState.bus24v_voltage;
		msg->telemetry_data[FC_24V_CURRENT_I] = Rocket_h.fcState.bus24v_current;
		msg->telemetry_data[FC_12V_VOLTAGE_I] = Rocket_h.fcState.bus12v_voltage;
		msg->telemetry_data[FC_12V_CURRENT_I] = Rocket_h.fcState.bus12v_current;
		msg->telemetry_data[FC_5V_VOLTAGE_I] = Rocket_h.fcState.bus5v_voltage;
		msg->telemetry_data[FC_5V_CURRENT_I] = Rocket_h.fcState.bus5v_current;
		msg->telemetry_data[FC_3V3_VOLTAGE_I] = Rocket_h.fcState.bus3v3_voltage;
		msg->telemetry_data[FC_3V3_CURRENT_I] = Rocket_h.fcState.bus3v3_current;
		xSemaphoreGive(Rocket_h.fcState_access);
	}
	else {
		return 1;
	}

	return 0;
}

// Unpack bay board message
// returns 0 if successful, 1 if the telemetry data could not be accessed or an invalid board id
// timeout_ticks: how many FreeRTOS ticks to wait before giving up
int unpack_bb_telemetry(TelemetryMessage *msg, uint8_t timeout_ticks) {
	switch(msg->board_id) {
	    case BOARD_BAY_1:
	    	if(xSemaphoreTake(Rocket_h.bb1State_access, timeout_ticks) == pdPASS) {
	    		Rocket_h.bb1State.vlv1_current = msg->telemetry_data[BB1_VLV1_CURRENT_I];
	    		Rocket_h.bb1State.vlv1_old = msg->telemetry_data[BB1_VLV1_OLD_I];
	    		Rocket_h.bb1State.vlv2_current = msg->telemetry_data[BB1_VLV2_CURRENT_I];
	    		Rocket_h.bb1State.vlv2_old = msg->telemetry_data[BB1_VLV2_OLD_I];
	    		Rocket_h.bb1State.vlv3_current = msg->telemetry_data[BB1_VLV3_CURRENT_I];
	    		Rocket_h.bb1State.vlv3_old = msg->telemetry_data[BB1_VLV3_OLD_I];
	    		Rocket_h.bb1State.vlv4_current = msg->telemetry_data[BB1_VLV4_CURRENT_I];
	    		Rocket_h.bb1State.vlv4_old = msg->telemetry_data[BB1_VLV4_OLD_I];
	    		Rocket_h.bb1State.vlv5_current = msg->telemetry_data[BB1_VLV5_CURRENT_I];
	    		Rocket_h.bb1State.vlv5_old = msg->telemetry_data[BB1_VLV5_OLD_I];
	    		Rocket_h.bb1State.vlv6_current = msg->telemetry_data[BB1_VLV6_CURRENT_I];
	    		Rocket_h.bb1State.vlv6_old = msg->telemetry_data[BB1_VLV6_OLD_I];
	    		Rocket_h.bb1State.vlv7_current = msg->telemetry_data[BB1_VLV7_CURRENT_I];
	    		Rocket_h.bb1State.vlv7_old = msg->telemetry_data[BB1_VLV7_OLD_I];
	    		Rocket_h.bb1State.pt1 = msg->telemetry_data[BB1_PT_1_I];
	    		Rocket_h.bb1State.pt2 = msg->telemetry_data[BB1_PT_2_I];
	    		Rocket_h.bb1State.pt3 = msg->telemetry_data[BB1_PT_3_I];
	    		Rocket_h.bb1State.pt4 = msg->telemetry_data[BB1_PT_4_I];
	    		Rocket_h.bb1State.pt5 = msg->telemetry_data[BB1_PT_5_I];
	    		Rocket_h.bb1State.pt6 = msg->telemetry_data[BB1_PT_6_I];
	    		Rocket_h.bb1State.pt7 = msg->telemetry_data[BB1_PT_7_I];
	    		Rocket_h.bb1State.pt8 = msg->telemetry_data[BB1_PT_8_I];
	    		Rocket_h.bb1State.pt9 = msg->telemetry_data[BB1_PT_9_I];
	    		Rocket_h.bb1State.pt10 = msg->telemetry_data[BB1_PT_10_I];
	    		Rocket_h.bb1State.tc1 = msg->telemetry_data[BB1_TC_1_I];
	    		Rocket_h.bb1State.tc2 = msg->telemetry_data[BB1_TC_2_I];
	    		Rocket_h.bb1State.tc3 = msg->telemetry_data[BB1_TC_3_I];
	    		Rocket_h.bb1State.tc4 = msg->telemetry_data[BB1_TC_4_I];
	    		Rocket_h.bb1State.tc5 = msg->telemetry_data[BB1_TC_5_I];
	    		Rocket_h.bb1State.tc6 = msg->telemetry_data[BB1_TC_6_I];
	    		Rocket_h.bb1State.imu1_A.XL_x = msg->telemetry_data[BB1_IMU1_X_I];
	    		Rocket_h.bb1State.imu1_A.XL_y = msg->telemetry_data[BB1_IMU1_Y_I];
	    		Rocket_h.bb1State.imu1_A.XL_z = msg->telemetry_data[BB1_IMU1_Z_I];
	    		Rocket_h.bb1State.imu1_W.G_y = msg->telemetry_data[BB1_IMU1_WP_I];
	    		Rocket_h.bb1State.imu1_W.G_x = msg->telemetry_data[BB1_IMU1_WR_I];
	    		Rocket_h.bb1State.imu1_W.G_z = msg->telemetry_data[BB1_IMU1_WY_I];
	    		Rocket_h.bb1State.imu2_A.XL_x = msg->telemetry_data[BB1_IMU2_X_I];
	    		Rocket_h.bb1State.imu2_A.XL_y = msg->telemetry_data[BB1_IMU2_Y_I];
	    		Rocket_h.bb1State.imu2_A.XL_z = msg->telemetry_data[BB1_IMU2_Z_I];
	    		Rocket_h.bb1State.imu2_W.G_y = msg->telemetry_data[BB1_IMU2_WP_I];
	    		Rocket_h.bb1State.imu2_W.G_x = msg->telemetry_data[BB1_IMU2_WR_I];
	    		Rocket_h.bb1State.imu2_W.G_z = msg->telemetry_data[BB1_IMU2_WY_I];
	    		Rocket_h.bb1State.bar1 = msg->telemetry_data[BB1_BAR_1_I];
	    		Rocket_h.bb1State.bar2 = msg->telemetry_data[BB1_BAR_2_I];
	    		Rocket_h.bb1State.bus24v_voltage = msg->telemetry_data[BB1_24V_VOLTAGE_I];
	    		Rocket_h.bb1State.bus24v_current = msg->telemetry_data[BB1_24V_CURRENT_I];
	    		Rocket_h.bb1State.bus12v_voltage = msg->telemetry_data[BB1_12V_VOLTAGE_I];
	    		Rocket_h.bb1State.bus12v_current = msg->telemetry_data[BB1_12V_CURRENT_I];
	    		Rocket_h.bb1State.bus5v_voltage = msg->telemetry_data[BB1_5V_VOLTAGE_I];
	    		Rocket_h.bb1State.bus5v_current = msg->telemetry_data[BB1_5V_CURRENT_I];
	    		Rocket_h.bb1State.bus3v3_voltage = msg->telemetry_data[BB1_3V3_VOLTAGE_I];
	    		Rocket_h.bb1State.bus3v3_current = msg->telemetry_data[BB1_3V3_CURRENT_I];
	    		Rocket_h.bb1State.timestamp = msg->timestamp;
	    		xSemaphoreGive(Rocket_h.bb1State_access);
	    	}
	    	else {
	    		return 1;
	    	}
	        break;
	    case BOARD_BAY_2:
	    	if(xSemaphoreTake(Rocket_h.bb2State_access, timeout_ticks) == pdPASS) {
	    		Rocket_h.bb2State.vlv1_current = msg->telemetry_data[BB2_VLV1_CURRENT_I];
	    		Rocket_h.bb2State.vlv1_old = msg->telemetry_data[BB2_VLV1_OLD_I];
	    		Rocket_h.bb2State.vlv2_current = msg->telemetry_data[BB2_VLV2_CURRENT_I];
	    		Rocket_h.bb2State.vlv2_old = msg->telemetry_data[BB2_VLV2_OLD_I];
	    		Rocket_h.bb2State.vlv3_current = msg->telemetry_data[BB2_VLV3_CURRENT_I];
	    		Rocket_h.bb2State.vlv3_old = msg->telemetry_data[BB2_VLV3_OLD_I];
	    		Rocket_h.bb2State.vlv4_current = msg->telemetry_data[BB2_VLV4_CURRENT_I];
	    		Rocket_h.bb2State.vlv4_old = msg->telemetry_data[BB2_VLV4_OLD_I];
	    		Rocket_h.bb2State.vlv5_current = msg->telemetry_data[BB2_VLV5_CURRENT_I];
	    		Rocket_h.bb2State.vlv5_old = msg->telemetry_data[BB2_VLV5_OLD_I];
	    		Rocket_h.bb2State.vlv6_current = msg->telemetry_data[BB2_VLV6_CURRENT_I];
	    		Rocket_h.bb2State.vlv6_old = msg->telemetry_data[BB2_VLV6_OLD_I];
	    		Rocket_h.bb2State.vlv7_current = msg->telemetry_data[BB2_VLV7_CURRENT_I];
	    		Rocket_h.bb2State.vlv7_old = msg->telemetry_data[BB2_VLV7_OLD_I];
	    		Rocket_h.bb2State.pt1 = msg->telemetry_data[BB2_PT_1_I];
	    		Rocket_h.bb2State.pt2 = msg->telemetry_data[BB2_PT_2_I];
	    		Rocket_h.bb2State.pt3 = msg->telemetry_data[BB2_PT_3_I];
	    		Rocket_h.bb2State.pt4 = msg->telemetry_data[BB2_PT_4_I];
	    		Rocket_h.bb2State.pt5 = msg->telemetry_data[BB2_PT_5_I];
	    		Rocket_h.bb2State.pt6 = msg->telemetry_data[BB2_PT_6_I];
	    		Rocket_h.bb2State.pt7 = msg->telemetry_data[BB2_PT_7_I];
	    		Rocket_h.bb2State.pt8 = msg->telemetry_data[BB2_PT_8_I];
	    		Rocket_h.bb2State.pt9 = msg->telemetry_data[BB2_PT_9_I];
	    		Rocket_h.bb2State.pt10 = msg->telemetry_data[BB2_PT_10_I];
	    		Rocket_h.bb2State.tc1 = msg->telemetry_data[BB2_TC_1_I];
	    		Rocket_h.bb2State.tc2 = msg->telemetry_data[BB2_TC_2_I];
	    		Rocket_h.bb2State.tc3 = msg->telemetry_data[BB2_TC_3_I];
	    		Rocket_h.bb2State.tc4 = msg->telemetry_data[BB2_TC_4_I];
	    		Rocket_h.bb2State.tc5 = msg->telemetry_data[BB2_TC_5_I];
	    		Rocket_h.bb2State.tc6 = msg->telemetry_data[BB2_TC_6_I];
	    		Rocket_h.bb2State.imu1_A.XL_x = msg->telemetry_data[BB2_IMU1_X_I];
	    		Rocket_h.bb2State.imu1_A.XL_y = msg->telemetry_data[BB2_IMU1_Y_I];
	    		Rocket_h.bb2State.imu1_A.XL_z = msg->telemetry_data[BB2_IMU1_Z_I];
	    		Rocket_h.bb2State.imu1_W.G_y = msg->telemetry_data[BB2_IMU1_WP_I];
	    		Rocket_h.bb2State.imu1_W.G_x = msg->telemetry_data[BB2_IMU1_WR_I];
	    		Rocket_h.bb2State.imu1_W.G_z = msg->telemetry_data[BB2_IMU1_WY_I];
	    		Rocket_h.bb2State.imu2_A.XL_x = msg->telemetry_data[BB2_IMU2_X_I];
	    		Rocket_h.bb2State.imu2_A.XL_y = msg->telemetry_data[BB2_IMU2_Y_I];
	    		Rocket_h.bb2State.imu2_A.XL_z = msg->telemetry_data[BB2_IMU2_Z_I];
	    		Rocket_h.bb2State.imu2_W.G_y = msg->telemetry_data[BB2_IMU2_WP_I];
	    		Rocket_h.bb2State.imu2_W.G_x = msg->telemetry_data[BB2_IMU2_WR_I];
	    		Rocket_h.bb2State.imu2_W.G_z = msg->telemetry_data[BB2_IMU2_WY_I];
	    		Rocket_h.bb2State.bar1 = msg->telemetry_data[BB2_BAR_1_I];
	    		Rocket_h.bb2State.bar2 = msg->telemetry_data[BB2_BAR_2_I];
	    		Rocket_h.bb2State.bus24v_voltage = msg->telemetry_data[BB2_24V_VOLTAGE_I];
	    		Rocket_h.bb2State.bus24v_current = msg->telemetry_data[BB2_24V_CURRENT_I];
	    		Rocket_h.bb2State.bus12v_voltage = msg->telemetry_data[BB2_12V_VOLTAGE_I];
	    		Rocket_h.bb2State.bus12v_current = msg->telemetry_data[BB2_12V_CURRENT_I];
	    		Rocket_h.bb2State.bus5v_voltage = msg->telemetry_data[BB2_5V_VOLTAGE_I];
	    		Rocket_h.bb2State.bus5v_current = msg->telemetry_data[BB2_5V_CURRENT_I];
	    		Rocket_h.bb2State.bus3v3_voltage = msg->telemetry_data[BB2_3V3_VOLTAGE_I];
	    		Rocket_h.bb2State.bus3v3_current = msg->telemetry_data[BB2_3V3_CURRENT_I];
	    		Rocket_h.bb2State.timestamp = msg->timestamp;
	    		xSemaphoreGive(Rocket_h.bb2State_access);
	    	}
	    	else {
	    		return 1;
	    	}
	        break;
	    case BOARD_BAY_3:
	    	if(xSemaphoreTake(Rocket_h.bb3State_access, timeout_ticks) == pdPASS) {
	    		Rocket_h.bb3State.vlv1_current = msg->telemetry_data[BB3_VLV1_CURRENT_I];
	    		Rocket_h.bb3State.vlv1_old = msg->telemetry_data[BB3_VLV1_OLD_I];
	    		Rocket_h.bb3State.vlv2_current = msg->telemetry_data[BB3_VLV2_CURRENT_I];
	    		Rocket_h.bb3State.vlv2_old = msg->telemetry_data[BB3_VLV2_OLD_I];
	    		Rocket_h.bb3State.vlv3_current = msg->telemetry_data[BB3_VLV3_CURRENT_I];
	    		Rocket_h.bb3State.vlv3_old = msg->telemetry_data[BB3_VLV3_OLD_I];
	    		Rocket_h.bb3State.vlv4_current = msg->telemetry_data[BB3_VLV4_CURRENT_I];
	    		Rocket_h.bb3State.vlv4_old = msg->telemetry_data[BB3_VLV4_OLD_I];
	    		Rocket_h.bb3State.vlv5_current = msg->telemetry_data[BB3_VLV5_CURRENT_I];
	    		Rocket_h.bb3State.vlv5_old = msg->telemetry_data[BB3_VLV5_OLD_I];
	    		Rocket_h.bb3State.vlv6_current = msg->telemetry_data[BB3_VLV6_CURRENT_I];
	    		Rocket_h.bb3State.vlv6_old = msg->telemetry_data[BB3_VLV6_OLD_I];
	    		Rocket_h.bb3State.vlv7_current = msg->telemetry_data[BB3_VLV7_CURRENT_I];
	    		Rocket_h.bb3State.vlv7_old = msg->telemetry_data[BB3_VLV7_OLD_I];
	    		Rocket_h.bb3State.pt1 = msg->telemetry_data[BB3_PT_1_I];
	    		Rocket_h.bb3State.pt2 = msg->telemetry_data[BB3_PT_2_I];
	    		Rocket_h.bb3State.pt3 = msg->telemetry_data[BB3_PT_3_I];
	    		Rocket_h.bb3State.pt4 = msg->telemetry_data[BB3_PT_4_I];
	    		Rocket_h.bb3State.pt5 = msg->telemetry_data[BB3_PT_5_I];
	    		Rocket_h.bb3State.pt6 = msg->telemetry_data[BB3_PT_6_I];
	    		Rocket_h.bb3State.pt7 = msg->telemetry_data[BB3_PT_7_I];
	    		Rocket_h.bb3State.pt8 = msg->telemetry_data[BB3_PT_8_I];
	    		Rocket_h.bb3State.pt9 = msg->telemetry_data[BB3_PT_9_I];
	    		Rocket_h.bb3State.pt10 = msg->telemetry_data[BB3_PT_10_I];
	    		Rocket_h.bb3State.tc1 = msg->telemetry_data[BB3_TC_1_I];
	    		Rocket_h.bb3State.tc2 = msg->telemetry_data[BB3_TC_2_I];
	    		Rocket_h.bb3State.tc3 = msg->telemetry_data[BB3_TC_3_I];
	    		Rocket_h.bb3State.tc4 = msg->telemetry_data[BB3_TC_4_I];
	    		Rocket_h.bb3State.tc5 = msg->telemetry_data[BB3_TC_5_I];
	    		Rocket_h.bb3State.tc6 = msg->telemetry_data[BB3_TC_6_I];
	    		Rocket_h.bb3State.imu1_A.XL_x = msg->telemetry_data[BB3_IMU1_X_I];
	    		Rocket_h.bb3State.imu1_A.XL_y = msg->telemetry_data[BB3_IMU1_Y_I];
	    		Rocket_h.bb3State.imu1_A.XL_z = msg->telemetry_data[BB3_IMU1_Z_I];
	    		Rocket_h.bb3State.imu1_W.G_y = msg->telemetry_data[BB3_IMU1_WP_I];
	    		Rocket_h.bb3State.imu1_W.G_x = msg->telemetry_data[BB3_IMU1_WR_I];
	    		Rocket_h.bb3State.imu1_W.G_z = msg->telemetry_data[BB3_IMU1_WY_I];
	    		Rocket_h.bb3State.imu2_A.XL_x = msg->telemetry_data[BB3_IMU2_X_I];
	    		Rocket_h.bb3State.imu2_A.XL_y = msg->telemetry_data[BB3_IMU2_Y_I];
	    		Rocket_h.bb3State.imu2_A.XL_z = msg->telemetry_data[BB3_IMU2_Z_I];
	    		Rocket_h.bb3State.imu2_W.G_y = msg->telemetry_data[BB3_IMU2_WP_I];
	    		Rocket_h.bb3State.imu2_W.G_x = msg->telemetry_data[BB3_IMU2_WR_I];
	    		Rocket_h.bb3State.imu2_W.G_z = msg->telemetry_data[BB3_IMU2_WY_I];
	    		Rocket_h.bb3State.bar1 = msg->telemetry_data[BB3_BAR_1_I];
	    		Rocket_h.bb3State.bar2 = msg->telemetry_data[BB3_BAR_2_I];
	    		Rocket_h.bb3State.bus24v_voltage = msg->telemetry_data[BB3_24V_VOLTAGE_I];
	    		Rocket_h.bb3State.bus24v_current = msg->telemetry_data[BB3_24V_CURRENT_I];
	    		Rocket_h.bb3State.bus12v_voltage = msg->telemetry_data[BB3_12V_VOLTAGE_I];
	    		Rocket_h.bb3State.bus12v_current = msg->telemetry_data[BB3_12V_CURRENT_I];
	    		Rocket_h.bb3State.bus5v_voltage = msg->telemetry_data[BB3_5V_VOLTAGE_I];
	    		Rocket_h.bb3State.bus5v_current = msg->telemetry_data[BB3_5V_CURRENT_I];
	    		Rocket_h.bb3State.bus3v3_voltage = msg->telemetry_data[BB3_3V3_VOLTAGE_I];
	    		Rocket_h.bb3State.bus3v3_current = msg->telemetry_data[BB3_3V3_CURRENT_I];
	    		Rocket_h.bb3State.timestamp = msg->timestamp;
	    		xSemaphoreGive(Rocket_h.bb3State_access);
	    	}
	    	else {
	    		return 1;
	    	}
	        break;
	    default:
	    	return 1;
	        break;
	}

	return 0;
}

// Unpack flight recorder telemetry message
// returns 0 if successful, 1 if the telemetry data could not be accessed
// timeout_ticks: how many FreeRTOS ticks to wait before giving up
int unpack_fr_telemetry(TelemetryMessage *msg, uint8_t timeout_ticks) {
	if(xSemaphoreTake(Rocket_h.frState_access, timeout_ticks) == pdPASS) {
		Rocket_h.frState.imu1_A.XL_x = msg->telemetry_data[FR_IMU1_X_I];
		Rocket_h.frState.imu1_A.XL_y = msg->telemetry_data[FR_IMU1_Y_I];
		Rocket_h.frState.imu1_A.XL_z = msg->telemetry_data[FR_IMU1_Z_I];
		Rocket_h.frState.imu1_W.G_y = msg->telemetry_data[FR_IMU1_WP_I];
		Rocket_h.frState.imu1_W.G_x = msg->telemetry_data[FR_IMU1_WR_I];
		Rocket_h.frState.imu1_W.G_z = msg->telemetry_data[FR_IMU1_WY_I];
		Rocket_h.frState.imu2_A.XL_x = msg->telemetry_data[FR_IMU2_X_I];
		Rocket_h.frState.imu2_A.XL_y = msg->telemetry_data[FR_IMU2_Y_I];
		Rocket_h.frState.imu2_A.XL_z = msg->telemetry_data[FR_IMU2_Z_I];
		Rocket_h.frState.imu2_W.G_y = msg->telemetry_data[FR_IMU2_WP_I];
		Rocket_h.frState.imu2_W.G_x = msg->telemetry_data[FR_IMU2_WR_I];
		Rocket_h.frState.imu2_W.G_z = msg->telemetry_data[FR_IMU2_WY_I];
		Rocket_h.frState.bar1 = msg->telemetry_data[FR_BAR_1_I];
		Rocket_h.frState.bar2 = msg->telemetry_data[FR_BAR_2_I];
		Rocket_h.frState.timestamp = msg->timestamp;
		xSemaphoreGive(Rocket_h.frState_access);
	}
	else {
		return 1;
	}

	return 0;
}

// Set a valve to a state and update its corresponding valve state
// Returns the state of the valve after setting it (this should be equal to desiredState, but in the case of an error, it will accurately reflect the current state of the valve)
Valve_State_t set_and_update_valve(Valve_Channel valve, Valve_State_t desiredState) {
	uint8_t vlverror = 0;
	 if(xSemaphoreTake(Rocket_h.fcValve_access, 5) == pdPASS) {
		 if(desiredState == Valve_Energized) {
			 VLV_En(Rocket_h.fcValves[valve]);
			 Rocket_h.fcValveStates[valve] = Valve_Energized;
		 }
		 else if(desiredState == Valve_Deenergized) {
			 VLV_Den(Rocket_h.fcValves[valve]);
			 Rocket_h.fcValveStates[valve] = Valve_Deenergized;
		 }
		 else {
			 // Error invalid state
			 vlverror = 1;
		 }
		 xSemaphoreGive(Rocket_h.fcValve_access);
	 }
	 else {
		 vlverror = 1;
	 }
	 if(!vlverror) {
		 return desiredState;
	 }
	 else {
		 return HAL_GPIO_ReadPin(Rocket_h.fcValves[valve].VLV_EN_GPIO_Port, Rocket_h.fcValves[valve].VLV_EN_GPIO_Pin);
	 }
}
