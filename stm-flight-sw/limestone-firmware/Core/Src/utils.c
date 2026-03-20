/*
 * utils.c
 *
 *  Created on: July 4th, 2025
 *      Author: felixfb
 */

#include "utils.h"

float current_sense_calc(uint16_t raw, uint16_t res, uint8_t divider) {
	float adc_v = (raw / 4095.0) * 3.3;
	if(divider) {
		adc_v = (adc_v * 5) / 3.0;
	}
	float shunt_v = adc_v / 50.0;
	return shunt_v / (res / 1000.0);
}

float bus_voltage_calc(uint16_t raw, uint32_t resA, uint32_t resB) {
	float res_divider = (resA + resB) / (float) resB;
	float adc_v = (raw / 4095.0) * 3.3;
	return adc_v * res_divider;
}

float PT_calc(PT_t PT_info, uint16_t raw) {
	float adc_v = (raw / 4095.0) * 3.3;
	float PT_v = adc_v * PT_DIVIDER;
	float PT_slope = (PT_info.max_V - PT_info.zero_V) / PT_info.pres_range;
	return (PT_v - PT_info.zero_V) / PT_slope;
}

void reset_board() {
	flush_flash_log_for_reset();

	NVIC_SystemReset(); // This should never return
}

// use_bat 1 to use battery source, 0 to use GSE
void PDB_source(uint8_t use_bat) {
	HAL_GPIO_WritePin(PDB_DIO2_GPIO_Port, PDB_DIO2_Pin, use_bat);
	if(use_bat) {
		log_message(FC_STAT_PDB_SWITCH_BAT, -1);
	}
	else {
		log_message(FC_STAT_PDB_SWITCH_GSE, -1);
	}
}

void COTS_supply(uint8_t enabled) {
	HAL_GPIO_WritePin(PDB_DIO1_GPIO_Port, PDB_DIO1_Pin, enabled);
	if(enabled) {
		log_message(FC_STAT_PDB_COTS_SUPPLY_ON, -1);
	}
	else {
		log_message(FC_STAT_PDB_COTS_SUPPLY_OFF, -1);
	}
}


int send_msg_to_device(Target_Device device, Message *msg, TickType_t wait) {
	if(is_server_running() <= 0) {
		return -1; // Server not running
	}
	int num_connected = num_devices(device);
	if(num_connected <= 0) {
		return -3; // Device not connected
	}
	for(int i = 0;i < num_connected;i++) {
		uint8_t tempbuffer[MAX_MSG_LEN];
		int buflen = serialize_message(msg, tempbuffer, MAX_MSG_LEN);
		if(buflen == -1) {
			continue; // Serialization error
		}
		int device_fd = get_device_fd(device, i);
		if(device_fd < 0) {
			continue;
		}
	    uint8_t *buffer = allocFromPool();
	    if(buffer) {
	        memcpy(buffer, tempbuffer, buflen);
	    	Raw_message rawmsg = {0};
	    	rawmsg.bufferptr = buffer;
	    	rawmsg.packet_len = buflen;
	    	rawmsg.connection_fd = device_fd;
	    	int result = server_send(&rawmsg, wait);
	    	if(result != 0) {
	    		freeFromPool(buffer);
	    	}
	    }
	}
	return 0;
}

int send_raw_msg_to_all_devices(Target_Device device, Raw_message *msg, TickType_t wait) {
	if(is_server_running() <= 0) {
		return -1; // Server not running
	}
	int num_connected = num_devices(device);
	if(num_connected <= 0) {
		return -3; // Device not connected
	}
	if(num_connected > 1) {
		for(int i = 1;i < num_connected;i++) {
			int device_fd = get_device_fd(device, i);
			if(device_fd < 0) {
				continue;
			}
		    uint8_t *buffer = allocFromPool();
		    if(buffer) {
		        memcpy(buffer, msg->bufferptr, msg->packet_len);
		    	Raw_message rawmsg = {0};
		    	rawmsg.bufferptr = buffer;
		    	rawmsg.packet_len = msg->packet_len;
		    	rawmsg.connection_fd = device_fd;
		    	int result = server_send(&rawmsg, wait);
		    	if(result != 0) {
		    		freeFromPool(buffer);
		    	}
		    }
		}
	}
	int device_fd = get_device_fd(device, 0);
	if(device_fd < 0) {
		return -3;
	}
	msg->connection_fd = device_fd;
	return server_send(msg, wait);
}

void set_valve_within(Valve_Channel valve, Valve_State_t desiredState) {
	Valve_State_t endState = set_and_update_valve(valve, desiredState);
	Message returnMsg = {0};
	returnMsg.type = MSG_VALVE_STATE;
	returnMsg.data.valve_state.valve_state = endState;
	returnMsg.data.valve_state.valve_id = generate_valve_id(BOARD_FC, valve);
	returnMsg.data.valve_state.timestamp = get_rtc_time();
	if(send_msg_to_device(LimeWire_d, &returnMsg, 5) != 0) {
		// Server not up, target device not connected, or txbuffer is full
	}
}
