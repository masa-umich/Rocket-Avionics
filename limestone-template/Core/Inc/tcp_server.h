/*
 * tcp_server.h
 *
 *  Created on: Oct 7, 2024
 *      Author: Jack
 */

#ifndef INC_TCP_SERVER_H_
#define INC_TCP_SERVER_H_

// Function to start the TCP server
void tcpserver_init(void);

/**** Send RESPONSE every time the client sends some data ******/
static void tcp_thread(void *arg);

#endif /* INC_TCP_SERVER_H_ */
