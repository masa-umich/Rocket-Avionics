/*
 * utils.c
 *
 *  Created on: July 4th, 2025
 *      Author: felixfb
 */

#include "utils.h"

void reset_board() {
	flush_flash_log_for_reset();

	NVIC_SystemReset(); // This should never return
}

// Send a LMP message over TCP
// wait is the number of ticks to wait for room in the txbuffer
// buffersize is the maximum size that it will take to serialize the LMP message, if this is 0, it will use the maximum possible message size to ensure proper serialization
// returns 0 on success, -1 if the client is not connected, -2 if there is no room in the txbuffer or space to allocate a buffer and -3 on a LMP serialization error
int send_msg_to_device(Message *msg, TickType_t wait) {
	if(is_client_connected() <= 0) {
		return -1; // Client not connected
	}
	uint8_t tempbuffer[MAX_MSG_LEN];
	int buflen = serialize_message(msg, tempbuffer, MAX_MSG_LEN);
	if(buflen == -1) {
		return -3; // Serialization error
	}
    uint8_t *buffer = allocFromPool();
    if(buffer) {
        memcpy(buffer, tempbuffer, buflen);
    	RawMessage rawmsg = {0};
    	rawmsg.bufferptr = buffer;
    	rawmsg.packet_len = buflen;
    	int result = client_send(&rawmsg, wait);
    	if(result != 0) {
    		freeFromPool(buffer);
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
