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

				    	if(send_raw_msg_to_all_devices(LimeWire_d, &msg, 2) == 0) {
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
				    			if(send_raw_msg_to_all_devices(get_valve_board(parsedmsg.data.valve_command.valve_id), &msg, 5) == 0) {
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

					    	if(send_raw_msg_to_all_devices(LimeWire_d, &msg, 5) == 0) {
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
				    				reset_board(); // No response since this will never return
				    				break;
				    			}
				    			case DEVICE_CMD_CLEAR_FLASH: {
				    				clear_flash(); // Sends its own response
				    				break;
				    			}
				    			case DEVICE_CMD_QUERY_FLASH: {
				    				Message dev_cmd_ack = {0};
				    				dev_cmd_ack.type = MSG_DEVICE_ACK;
				    				dev_cmd_ack.data.device_ack.board_id = BOARD_FC;
				    				dev_cmd_ack.data.device_ack.cmd_id = DEVICE_CMD_QUERY_FLASH;
				    				log_flash_storage(dev_cmd_ack.data.device_ack.payload, MAX_ACK_PAYLOAD_SIZE);
					      			if(send_msg_to_device(LimeWire_d, &dev_cmd_ack, 5, strlen(dev_cmd_ack.data.device_ack.payload) + 3 + DEVICE_COMMAND_ACK_HEADER_SIZE) != 0) {
					      				// Server not up, target device not connected, or txbuffer is full
					      			}
				    				break;
				    			}
				    			case DEVICE_CMD_PDB_SRC_GSE: {
				    				PDB_source(0);
				    				Message dev_cmd_ack = {0};
				    				dev_cmd_ack.type = MSG_DEVICE_ACK;
				    				dev_cmd_ack.data.device_ack.board_id = BOARD_FC;
				    				dev_cmd_ack.data.device_ack.cmd_id = DEVICE_CMD_PDB_SRC_GSE;
				    				strncpy(dev_cmd_ack.data.device_ack.payload, FC_PDB_SRC_GSE_ACK_MSG, sizeof(dev_cmd_ack.data.device_ack.payload));
				    				dev_cmd_ack.data.device_ack.payload[sizeof(dev_cmd_ack.data.device_ack.payload) - 1] = '\0';
					      			if(send_msg_to_device(LimeWire_d, &dev_cmd_ack, 5, strlen(dev_cmd_ack.data.device_ack.payload) + 3 + DEVICE_COMMAND_ACK_HEADER_SIZE) != 0) {
					      				// Server not up, target device not connected, or txbuffer is full
					      			}
				    				break;
				    			}
				    			case DEVICE_CMD_PDB_SRC_BAT: {
				    				PDB_source(1);
				    				Message dev_cmd_ack = {0};
				    				dev_cmd_ack.type = MSG_DEVICE_ACK;
				    				dev_cmd_ack.data.device_ack.board_id = BOARD_FC;
				    				dev_cmd_ack.data.device_ack.cmd_id = DEVICE_CMD_PDB_SRC_BAT;
				    				strncpy(dev_cmd_ack.data.device_ack.payload, FC_PDB_SRC_BAT_ACK_MSG, sizeof(dev_cmd_ack.data.device_ack.payload));
				    				dev_cmd_ack.data.device_ack.payload[sizeof(dev_cmd_ack.data.device_ack.payload) - 1] = '\0';
					      			if(send_msg_to_device(LimeWire_d, &dev_cmd_ack, 5, strlen(dev_cmd_ack.data.device_ack.payload) + 3 + DEVICE_COMMAND_ACK_HEADER_SIZE) != 0) {
					      				// Server not up, target device not connected, or txbuffer is full
					      			}
				    				break;
				    			}
				    			case DEVICE_CMD_PDB_COTS_OFF: {
				    				COTS_supply(0);
				    				Message dev_cmd_ack = {0};
				    				dev_cmd_ack.type = MSG_DEVICE_ACK;
				    				dev_cmd_ack.data.device_ack.board_id = BOARD_FC;
				    				dev_cmd_ack.data.device_ack.cmd_id = DEVICE_CMD_PDB_COTS_OFF;
				    				strncpy(dev_cmd_ack.data.device_ack.payload, FC_PDB_COTS_OFF_ACK_MSG, sizeof(dev_cmd_ack.data.device_ack.payload));
				    				dev_cmd_ack.data.device_ack.payload[sizeof(dev_cmd_ack.data.device_ack.payload) - 1] = '\0';
					      			if(send_msg_to_device(LimeWire_d, &dev_cmd_ack, 5, strlen(dev_cmd_ack.data.device_ack.payload) + 3 + DEVICE_COMMAND_ACK_HEADER_SIZE) != 0) {
					      				// Server not up, target device not connected, or txbuffer is full
					      			}
				    				break;
				    			}
				    			case DEVICE_CMD_PDB_COTS_ON: {
				    				COTS_supply(1);
				    				Message dev_cmd_ack = {0};
				    				dev_cmd_ack.type = MSG_DEVICE_ACK;
				    				dev_cmd_ack.data.device_ack.board_id = BOARD_FC;
				    				dev_cmd_ack.data.device_ack.cmd_id = DEVICE_CMD_PDB_COTS_ON;
				    				strncpy(dev_cmd_ack.data.device_ack.payload, FC_PDB_COTS_ON_ACK_MSG, sizeof(dev_cmd_ack.data.device_ack.payload));
				    				dev_cmd_ack.data.device_ack.payload[sizeof(dev_cmd_ack.data.device_ack.payload) - 1] = '\0';
					      			if(send_msg_to_device(LimeWire_d, &dev_cmd_ack, 5, strlen(dev_cmd_ack.data.device_ack.payload) + 3 + DEVICE_COMMAND_ACK_HEADER_SIZE) != 0) {
					      				// Server not up, target device not connected, or txbuffer is full
					      			}
				    				break;
				    			}
				    			case DEVICE_CMD_BUILD_INFO: {
				    				Message dev_cmd_ack = {0};
				    				dev_cmd_ack.type = MSG_DEVICE_ACK;
				    				dev_cmd_ack.data.device_ack.board_id = BOARD_FC;
				    				dev_cmd_ack.data.device_ack.cmd_id = DEVICE_CMD_BUILD_INFO;
				    				log_message(STAT_VERSION_INFO, -1);
				    				memcpy(dev_cmd_ack.data.device_ack.payload, STAT_VERSION_INFO + 4, sizeof(STAT_VERSION_INFO) - 4);
					      			if(send_msg_to_device(LimeWire_d, &dev_cmd_ack, 5, strlen(dev_cmd_ack.data.device_ack.payload) + 3 + DEVICE_COMMAND_ACK_HEADER_SIZE) != 0) {
					      				// Server not up, target device not connected, or txbuffer is full
					      			}
				    				break;
				    			}
				    			default: {
				    				break;
				    			}
				    		}
				    	}
				    	else {
			    			// Relay to other boards
			    			if(send_raw_msg_to_all_devices(parsedmsg.data.device_command.board_id, &msg, 5) == 0) {
					    		// Continue to prevent freeing memory we're still using
					    		continue;
			    			}
			    			else {
			    				// Server not up, target device not connected, or txbuffer is full
			    			}
				    	}
				    	break;
				    }
				    case MSG_DEVICE_ACK: {
		    			if(send_raw_msg_to_all_devices(LimeWire_d, &msg, 5) == 0) {
				    		// Continue to prevent freeing memory we're still using
				    		continue;
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
