/*
 * network-processing.c
 *
 *  Created on: Oct 29, 2025
 *      Author: felix
 */

#include "network-processing.h"

void ProcessPackets(void *argument) {
	// Started processing thread
	log_message(STAT_PACKET_TASK_STARTED, -1);
	for(;;) {
		Raw_message msg = {0};
		int read_stat = server_read(&msg, 50);
		if(read_stat >= 0) {
			// The way the msg.bufferptr memory is handled past this point is that any result that doesn't relay msg to a different destination should NOT continue early, any result that does relay msg should continue early
			Message parsedmsg = {0};
			if(deserialize_message(msg.bufferptr, msg.packet_len, &parsedmsg) > 0) {
				switch(parsedmsg.type) {
				    case MSG_TELEMETRY: {
				        // Save and relay to Limewire
				    	if(parsedmsg.data.telemetry.board_id == BOARD_FR) {
				    		if(unpack_fr_telemetry(&(parsedmsg.data.telemetry), 5)) {
				    			// Failed to save data
				    			log_message(ERR_SAVE_INCOMING_TELEM, FC_ERR_TYPE_INCOMING_TELEM);
				    		}
				    	}
				    	else {
					    	if(unpack_bb_telemetry(&(parsedmsg.data.telemetry), 5)) {
					    		// Failed to save data
					    		log_message(ERR_SAVE_INCOMING_TELEM, FC_ERR_TYPE_INCOMING_TELEM);
					    	}
				    	}

				    	if(send_raw_msg_to_device(LimeWire_d, &msg, 2) == 0) {
				    		// Continue to prevent freeing memory we're still using
				    		continue;
				    	}
				    	else {
				    	  	// Server not up, target device not connected, or txbuffer is full
				    	}
				        break;
				    }
				    case MSG_VALVE_COMMAND: {
				    	if(check_valve_id(parsedmsg.data.valve_command.valve_id)) {
				    		if(get_valve_board(parsedmsg.data.valve_command.valve_id) == BOARD_FC) {
				    			// Do valve command and send state message
				    			Valve_State_t endState = set_and_update_valve(get_valve(parsedmsg.data.valve_command.valve_id), parsedmsg.data.valve_command.valve_state);
				    			Message returnMsg = {0};
				    			returnMsg.type = MSG_VALVE_STATE;
				    			returnMsg.data.valve_state.valve_state = endState;
				    			returnMsg.data.valve_state.valve_id = parsedmsg.data.valve_command.valve_id;
				    			returnMsg.data.valve_state.timestamp = get_rtc_time();
				      			if(send_msg_to_device(LimeWire_d, &returnMsg, 5, MAX_VALVE_STATE_MSG_SIZE + 5) != 0) {
				      				// Server not up, target device not connected, or txbuffer is full
				      			}
				    		}
				    		else {
				    			// Relay to Bay Boards
				    			if(send_raw_msg_to_device(get_valve_board(parsedmsg.data.valve_command.valve_id), &msg, 5) == 0) {
						    		// Continue to prevent freeing memory we're still using
						    		continue;
				    			}
				    			else {
				    				// Server not up, target device not connected, or txbuffer is full
				    			}
				    		}
				    	}
				    	else {
				    		// Invalid valve id
				    		log_message(ERR_PROCESS_VLV_CMD_BADID, FC_ERR_TYPE_BAD_VLVID);
				    	}
				        break;
				    }
				    case MSG_VALVE_STATE: {
				        // Save and relay to Limewire
				    	if(check_valve_id(parsedmsg.data.valve_state.valve_id)) {
					    	switch(get_valve_board(parsedmsg.data.valve_state.valve_id)) {
					    	    case BOARD_BAY_1: {
					    	  	  	if(xSemaphoreTake(Rocket_h.bb1Valve_access, 5) == pdPASS) {
					    	  	  		Rocket_h.bb1ValveStates[get_valve(parsedmsg.data.valve_state.valve_id)] = parsedmsg.data.valve_state.valve_state;
					    	  	  		xSemaphoreGive(Rocket_h.bb1Valve_access);
					    	  	  	}
					    	        break;
					    	    }
					    	    case BOARD_BAY_2: {
					    	  	  	if(xSemaphoreTake(Rocket_h.bb2Valve_access, 5) == pdPASS) {
					    	  	  		Rocket_h.bb2ValveStates[get_valve(parsedmsg.data.valve_state.valve_id)] = parsedmsg.data.valve_state.valve_state;
					    	  	  		xSemaphoreGive(Rocket_h.bb2Valve_access);
					    	  	  	}
					    	        break;
					    	    }
					    	    case BOARD_BAY_3: {
					    	  	  	if(xSemaphoreTake(Rocket_h.bb3Valve_access, 5) == pdPASS) {
					    	  	  		Rocket_h.bb3ValveStates[get_valve(parsedmsg.data.valve_state.valve_id)] = parsedmsg.data.valve_state.valve_state;
					    	  	  		xSemaphoreGive(Rocket_h.bb3Valve_access);
					    	  	  	}
					    	        break;
					    	    }
					    	    default: {
					    	        // The flight computer should not receive valve state messages for its own valves
					    	        break;
					    	    }
					    	}

					    	if(send_raw_msg_to_device(LimeWire_d, &msg, 5) == 0) {
					    		// Continue to prevent freeing memory we're still using
					    		continue;
					    	}
					    	else {
					    	  	// Server not up, target device not connected, or txbuffer is full
					    	}
				    	}
				    	else {
				    		// Invalid valve id
				    		log_message(ERR_PROCESS_VLV_STATE_BADID, FC_ERR_TYPE_BAD_VLVID);
				    	}
				        break;
				    }
				    case MSG_DEVICE_COMMAND: {
				    	if(parsedmsg.data.device_command.board_id == BOARD_FC) {
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
				    			case DEVICE_CMD_PDB_SRC_GSE: {
				    				PDB_source(0);
				    				break;
				    			}
				    			case DEVICE_CMD_PDB_SRC_BAT: {
				    				PDB_source(1);
				    				break;
				    			}
				    			case DEVICE_CMD_PDB_COTS_OFF: {
				    				COTS_supply(0);
				    				break;
				    			}
				    			case DEVICE_CMD_PDB_COTS_ON: {
				    				COTS_supply(1);
				    				break;
				    			}
				    			default: {
				    				break;
				    			}
				    		}
				    	}
				    	else {
			    			// Relay to other boards
			    			if(send_raw_msg_to_device(parsedmsg.data.device_command.board_id, &msg, 5) == 0) {
					    		// Continue to prevent freeing memory we're still using
					    		continue;
			    			}
			    			else {
			    				// Server not up, target device not connected, or txbuffer is full
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
				log_message(ERR_UNKNOWN_LMP_PACKET, FC_ERR_TYPE_UNKNOWN_LMP);
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
