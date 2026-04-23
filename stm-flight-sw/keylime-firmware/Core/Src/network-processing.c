/*
 * network-processing.c
 *
 *  Created on: Nov 9, 2025
 *      Author: felix
 */

#include "network-processing.h"

void ProcessPackets(void *argument) {
	// Started processing thread
	log_message(STAT_PACKET_TASK_STARTED, -1);
	for(;;) {
		RawMessage msg = {0};
		int read_stat = client_receive(&msg, 1000);
		if(read_stat >= 0) {
			// The way the msg.bufferptr memory is handled past this point is that any result that doesn't relay msg to a different destination should NOT continue early, any result that does relay msg should continue early
			Message parsedmsg = {0};
			if(deserialize_message(msg.bufferptr, msg.packet_len, &parsedmsg) > 0) {
				switch(parsedmsg.type) {
				    case MSG_DEVICE_COMMAND: {
				    	if(parsedmsg.data.device_command.board_id == BOARD_FR) {
				    		// Process device command
				    		switch(parsedmsg.data.device_command.cmd_id) {
				    			case DEVICE_CMD_RESET: {
				    				reset_board();
				    				break;
				    			}
				    			case DEVICE_CMD_CLEAR_FLASH: {
				    				clear_flash();
				    				break;
				    			}
				    			case DEVICE_CMD_QUERY_FLASH: {
				    				Message dev_cmd_ack = {0};
				    				dev_cmd_ack.type = MSG_DEVICE_ACK;
				    				dev_cmd_ack.data.device_ack.board_id = BOARD_FR;
				    				dev_cmd_ack.data.device_ack.cmd_id = DEVICE_CMD_QUERY_FLASH;
				    				log_flash_storage(dev_cmd_ack.data.device_ack.payload, MAX_ACK_PAYLOAD_SIZE);
					      			if(send_msg_to_device(&dev_cmd_ack, 5) != 0) {
					      				// Client not up, target device not connected, or txbuffer is full
					      			}
				    				break;
				    			}
				    			case DEVICE_CMD_BUILD_INFO: {
				    				Message dev_cmd_ack = {0};
				    				dev_cmd_ack.type = MSG_DEVICE_ACK;
				    				dev_cmd_ack.data.device_ack.board_id = BOARD_FR;
				    				dev_cmd_ack.data.device_ack.cmd_id = DEVICE_CMD_BUILD_INFO;
				    				log_message(STAT_VERSION_INFO, -1);
				    				strlcpy(dev_cmd_ack.data.device_ack.payload, STAT_VERSION_INFO + 4, sizeof(dev_cmd_ack.data.device_ack.payload));
					      			if(send_msg_to_device(&dev_cmd_ack, 5) != 0) {
					      				// Server not up, target device not connected, or txbuffer is full
					      			}
				    				break;
				    			}
				    			default: {
				    				break;
				    			}
				    		}
				    	}
				    	break;
				    }
				    default: {
				        break;
				    }
				}
			}
			else {
				// Unknown message type
				log_message(ERR_UNKNOWN_LMP_PACKET, FR_ERR_TYPE_UNKNOWN_LMP);
			}
			freeFromPool(msg.bufferptr);
		}
		else if(read_stat == -1) {
			// Server down, delay to prevent taking CPU time from other tasks
			osDelay(100);
		}
		else {
			// Timeout on waiting for messages
		}
	}
}
