/*
 * client.h
 *
 *  Created on: Aug 13, 2025
 *      Author: felix
 */

#ifndef INC_CLIENT_H_
#define INC_CLIENT_H_

#define TCP_KEEP_ALIVE_IDLE	5
#define TCP_KEEP_ALIVE_INTERVAL	3
#define TCP_KEEP_ALIVE_COUNT 3
#define MAX_MSG_LEN 300
#define RETRY_DELAY_MS 100

typedef struct {
	uint8_t *bufferptr;
	int packet_len; // length of the packet
} RawMessage;

int client_init(ip4_addr_t *fcaddr);

int client_stop();

int client_reinit();

int client_send(RawMessage *msg, TickType_t block);

int client_receive(RawMessage *msg, TickType_t block);

void client_receive_thread(void *arg);

void client_send_thread(void *arg);

int is_client_running();

#endif /* INC_CLIENT_H_ */
