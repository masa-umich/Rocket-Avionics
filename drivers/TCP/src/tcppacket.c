/*
 * tcppacket.c
 *
 *  Created on: March 10, 2024
 *      Author: jackmh
 */
#include "../inc/tcppacket.h"
#include <stdint.h>

// Externs for incoming packet commands
extern void fsm_transition(int transition);
extern void sw_reset(void);
extern void set_bootloader(void);
extern void get_git_hash(void);
extern void valve_control(uint32_t valve, uint32_t state);
extern void config_calibration(uint64_t* calibration_data);

/* Decodes an incoming TCP packet and creates an outgoing packet if needed */
int rx_tcppacket_parse(struct packet *incoming_packet, struct packet *outgoing_packet) {
    int ack = 0x05; // Acknowledge bit
    // Get the incoming packet data in bytes
    char *data = incoming_packet->packet;
    int len = incoming_packet->packet_len; // We don't use this, but it's good to have
    // Get the first byte of the incoming packet
    int header = data[0];
    // Switch statement based on the first byte of the incoming packet
    switch (header) {
        case 0x01: // Telemetry data
            // Why are we getting a telemetry packet? We're supposed to be sending those?
            // Actually if we're flight computer we should send back these packets to the PC
        	break;
        case 0x02: // Command packet
            // Get command type
            int command = data[1];
            // Switch statement based on the command type
            switch (command) {
                case 0x00: // FSM abort
                    fsm_transition(0);
                    break;
                case 0x01: // FSM reset
                    fsm_transition(1);
                    break;
                case 0x02: // FSM bootloader
                    fsm_transition(2);
                    break;
                case 0x03: // FSM arm
                    fsm_transition(3);
                    break;
                case 0x04: // FSM disarm
                    fsm_transition(4);
                    break;
                case 0x05: // FSM launch
                    fsm_transition(5);
                    break;
                case 0x06: // FSM apogee
                    fsm_transition(6);
                    break;
                case 0x07: // SW reset
                    sw_reset();
                    break;
                case 0x08: // Set bootloader
                    set_bootloader();
                    break;
                case 0x09: // Get git hash
                    get_git_hash();
                    break;
                case 0x0A: // Heartbeat
                    // Do nothing
                    break;
                default:
                    // Invalid command
                    // We will tell the client that the command we thought it was about in the acknoledgment packet below
                    return -1;
            }
            // Make a success packet
            //tcppacket_encode((ack | header | command), 3, outgoing_packet);
            break;
        case 0x03: // Valves
            data++; // Skip the header byte

            uint32_t valve_selection = *(uint32_t*)data; // Get the next 4 bytes as the valve selection

            data += 4; // Skip the 4 bytes of valve selection

            uint32_t valve_state = *(uint32_t*)data; // Get the next 4 bytes as the valve state

            valve_control(valve_selection, valve_state);

            //tcppacket_encode((ack | header | valve_selection | valve_state), 10, outgoing_packet); // Make a ack packet with the valve selection and state

            break;
        case 0x04: // Config calibration
            data++; // Skip the header byte
            config_calibration((uint64_t*)data); // Pass the data to the calibration function
            //tcppacket_encode((ack | header), 2, outgoing_packet); // Make a ack packet with the header byte
            break;
        default:
            // Invalid command
            // Make an error packet with a code and what we thought was sent

            //char* return_message = strncat(ack << 8 | header);
        	//tcppacket_encode(return_message, (2+len), outgoing_packet);
            return -1;
    }
};

/* Encodes data into an outgoing TCP packet */
int tcppacket_encode(char* message, int message_length, struct packet *outgoing_packet) {
    // Copy the message into the outgoing packet
    strcpy(outgoing_packet->packet, message);
    // Set the length of the outgoing packet
    outgoing_packet->packet_len = message_length;
    return 0;
};

/* Telemetry to packet */
int tx_telemetry_encode(void) {

};
