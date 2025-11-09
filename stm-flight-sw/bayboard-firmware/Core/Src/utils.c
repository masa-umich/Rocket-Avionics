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

// Send a LMP message over TCP
// wait is the number of ticks to wait for room in the txbuffer
// buffersize is the maximum size that it will take to serialize the LMP message, if this is 0, it will use the maximum possible message size to ensure proper serialization
// returns 0 on success, -1 if the client is not connected, -2 if there is no room in the txbuffer or space to allocate a buffer and -3 on a LMP serialization error
int send_msg_to_device(Message *msg, TickType_t wait, size_t buffersize) {
	if(buffersize == 0) {
		buffersize = MAX_MSG_LEN;
	}
	if(is_client_connected() <= 0) {
		return -1; // Client not connected
	}
	uint8_t tempbuffer[buffersize];
	int buflen = serialize_message(msg, tempbuffer, buffersize);
	if(buflen == -1) {
		return -3; // Serialization error
	}
    uint8_t *buffer = malloc(buflen);
    if(buffer) {
        memcpy(buffer, tempbuffer, buflen);
    	RawMessage rawmsg = {0};
    	rawmsg.bufferptr = buffer;
    	rawmsg.packet_len = buflen;
    	int result = client_send(&rawmsg, wait);
    	if(result != 0) {
    		free(buffer);
    	}
    	return result;
    }
    return -2;
}

// Send a message over TCP
// wait is the number of ticks to wait for room in the txbuffer
// returns 0 on success, -1 if the client is not connected, -2 if there is no room in the txbuffer
// IMPORTANT: you are responsible for freeing the buffer in msg if this function returns something other than 0
int send_raw_msg_to_device(RawMessage *msg, TickType_t wait) {
	if(is_client_connected() <= 0) {
		return -1; // Client not running
	}
	return client_send(msg, wait);
}
