/*
 * tcpserver.h
 *
 *  Created on: Jan 16, 2024
 *      Author: evanm
 */

#ifndef INC_TCPSERVER_H_
#define INC_TCPSERVER_H_

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include <string.h>
#include <stdbool.h>

#include "gitcommit.h"

typedef struct netconn	netconn;

#define MAX_MSG_LEN 200

#define MAX_CONN_NUM 7

#define SERVER_PORT 50000

static netconn* connections[MAX_CONN_NUM] = { NULL };

static struct netconn *conn;

/*
 * Starts the TCP server
 */
void server_init(void);

/*
 * Sets the server in connection mode
 * When a connection is established, an ID is generated and pushed to the connections list for use
 * Server reenters connection mode
 */
static void server_waitForClientConnection(void *arg);

/*
 * Adds netconn to connections list and assigns a free ID to a netconn
 * RETURNS assigned ID, -1 for a rejected connection
 */
static int8_t server_addConnection(void);

/*
 * Removes netconn from connections list and frees ID for a new netconn to use
 */
static void server_removeConnection(u8_t ID);

static void server_recv(void *arg);

static void server_broadcast(void *arg);


#endif /* INC_TCPSERVER_H_ */
