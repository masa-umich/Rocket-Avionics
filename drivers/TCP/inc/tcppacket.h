/*
 * tcppacket.h
 *
 *  Created on: March 10, 2024
 *      Author: jackmh
 */

struct packet {
    char *packet;
    int packet_len;
};

#ifndef TCPPACKET_H_
#define TCPPACKET_H_

/* Decodes an incoming TCP packet and creates an outgoing packet if needed */
int tcppacket_parse(struct packet *incoming_packet, struct packet *outgoing_packet);

/* Encodes data into an outgoing TCP packet */
int tcppacket_encode(char* message, int message_length, struct packet *outgoing_packet);

#endif /* TCPPACKET_H_ */
