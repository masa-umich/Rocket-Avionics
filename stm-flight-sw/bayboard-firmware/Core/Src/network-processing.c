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
				    case MSG_VALVE_COMMAND: {
				    	if(check_valve_id(parsedmsg.data.valve_command.valve_id)) {
				    		if(get_valve_board(parsedmsg.data.valve_command.valve_id) == bb_num) {
				    			// Do valve command and send state message
				    			Valve_State_t endState = set_and_update_valve(get_valve(parsedmsg.data.valve_command.valve_id), parsedmsg.data.valve_command.valve_state);
				    			Message returnMsg = {0};
				    			returnMsg.type = MSG_VALVE_STATE;
				    			returnMsg.data.valve_state.valve_state = endState;
				    			returnMsg.data.valve_state.valve_id = parsedmsg.data.valve_command.valve_id;
				    			returnMsg.data.valve_state.timestamp = get_rtc_time();
				      			if(send_msg_to_device(&returnMsg, 5, MAX_VALVE_STATE_MSG_SIZE + 5) != 0) {
				      				// Server not up, target device not connected, or txbuffer is full
				      			}
				    		}
				    		else {
				    			// Not a message for this bay board. This is bad because it means the flight computer is malfunctioning
				    			log_message(ERR_PROCESS_VLV_CMD_BADID, BB_ERR_TYPE_BAD_VLVID);
				    		}
				    	}
				    	else {
				    		// Invalid valve id
				    		log_message(ERR_PROCESS_VLV_CMD_BADID, BB_ERR_TYPE_BAD_VLVID);
				    	}
				        break;
				    }
				    case MSG_DEVICE_COMMAND: {
				    	if(parsedmsg.data.device_command.board_id == bb_num) {
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
				    				log_flash_storage();
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
				log_message(ERR_UNKNOWN_LMP_PACKET, BB_ERR_TYPE_UNKNOWN_LMP);
			}
			free(msg.bufferptr);
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
