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
	flush_flash_log();

	NVIC_SystemReset(); // This should never return
}

// use_bat 1 to use battery source, 0 to use GSE
void PDB_source(uint8_t use_bat) {
	HAL_GPIO_WritePin(PDB_DIO1_GPIO_Port, PDB_DIO1_Pin, use_bat);
	if(use_bat) {
		log_message(FC_STAT_PDB_SWITCH_BAT, -1);
	}
	else {
		log_message(FC_STAT_PDB_SWITCH_GSE, -1);
	}
}

void COTS_supply(uint8_t enabled) {
	HAL_GPIO_WritePin(PDB_DIO2_GPIO_Port, PDB_DIO2_Pin, enabled);
	if(enabled) {
		log_message(FC_STAT_PDB_COTS_SUPPLY_ON, -1);
	}
	else {
		log_message(FC_STAT_PDB_COTS_SUPPLY_OFF, -1);
	}
}

// Send a LMP message over TCP
// wait is the number of ticks to wait for room in the txbuffer
// buffersize is the maximum size that it will take to serialize the LMP message, if this is 0, it will use the maximum possible message size to ensure proper serialization
// returns 0 on success, -1 if the server is not up, -2 if there is no room in the txbuffer or space to allocate a buffer, -3 if the target device is not connected, and -4 on a LMP serialization error
int send_msg_to_device(Target_Device device, Message *msg, TickType_t wait, size_t buffersize) {
	if(buffersize == 0) {
		buffersize = MAX_MSG_LEN;
	}
	if(is_server_running() <= 0) {
		return -1; // Server not running
	}
	int devicefd = get_device_fd(device);
	if(devicefd < 0) {
		return -3; // Device not connected
	}
	uint8_t tempbuffer[buffersize];
	int buflen = serialize_message(msg, tempbuffer, buffersize);
	if(buflen == -1) {
		return -4; // Serialization error
	}
    uint8_t *buffer = malloc(buflen);
    if(buffer) {
        memcpy(buffer, tempbuffer, buflen);
    	Raw_message rawmsg = {0};
    	rawmsg.bufferptr = buffer;
    	rawmsg.packet_len = buflen;
    	rawmsg.connection_fd = devicefd;
    	int result = server_send(&rawmsg, wait);
    	if(result != 0) {
    		free(buffer);
    	}
    	return result;
    }
    return -2;
}

// Send a message over TCP
// wait is the number of ticks to wait for room in the txbuffer
// returns 0 on success, -1 if the server is not up, -2 if there is no room in the txbuffer, -3 if the target device is not connected
// IMPORTANT: you are responsible for freeing the buffer in msg if this function returns something other than 0. This function does not
// take "ownership" of the message object
int send_raw_msg_to_device(Target_Device device, Raw_message *msg, TickType_t wait) {
	if(is_server_running() <= 0) {
		return -1; // Server not running
	}
	int devicefd = get_device_fd(device);
	if(devicefd < 0) {
		return -3; // Device not connected
	}
	msg->connection_fd = devicefd;
	return server_send(msg, wait);
}
