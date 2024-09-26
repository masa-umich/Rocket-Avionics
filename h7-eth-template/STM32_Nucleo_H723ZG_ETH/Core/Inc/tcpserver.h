/*
 * tcpserver.h
 *
 *  Created on: Jan 16, 2024
 *      Author: evanm
 */
#pragma once
#ifndef INC_TCPSERVER_H_
#define INC_TCPSERVER_H_

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include "lwip.h"
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "tsqueue.h"
#include "gitcommit.h"

#define MAX_MSG_LEN 300

#define MAX_CONN_NUM 8

#define SERVER_PORT 50000

#define ALL_CONNECTIONS -1

struct message {
	int connfd;
	char* buf;
	int len;
};

typedef enum {
	PARSE_OK, PARSE_ERR,
} parse_t;


parse_t devparse(char *data, u16_t len, char *response, u16_t *ret_len,
		int connfd);

/*
 * Starts the TCP server
 * Creates a listen thread, a recv thread, and a send thread
 */
void server_init(void);

/*
 * Sets the server in listen mode
 * When a connection is established, a fd is generated and pushed to the connections list for use
 */
void server_listen(void *arg);

void server_recv(void *arg);

void server_send(void *args);

void server_sendMsg(int dest, char* msg, int len);

void server_retrieveMsg(struct message* msg);

/*
 * Adds socket to connections list
 */
void server_addConnection(int connfd);

/*
 * Removes socket from connections list and frees fd for a new socket to use
 */
 void server_removeConnection(int connfd);

 void server_removeAllConnections(void);

 void server_setFDs(fd_set* rfds);

 void server_handleRecv(fd_set* rfds);

 void msgFreeCallback(void * data);


#endif /* INC_TCPSERVER_H_ */
