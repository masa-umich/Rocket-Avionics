/*
 * tcpserver.c
 *
 *  Created on: Jan 16, 2024
 *      Author: evanm
 */
#include "../inc/tcpserver.h"

extern RTC_HandleTypeDef hrtc;

List* rxMsgBuffer;
List* txMsgBuffer;

extern ip4_addr_t ipaddr;

int connections[MAX_CONN_NUM];

sys_mutex_t* conn_mu;

#ifndef DEVPARSE
#define DEVPARSE
parse_t devparse(char *data, u16_t len, char *response, u16_t *ret_len,
		int connfd) {
	/*
	 *  Data stores len number of bytes
	 *
	 *  Packet structure:
	 *  	6 bits: opcode
	 */

	char *str;

	if (len > 0) {

		u8_t opcode = (*data); //>> 2); // get opcode from first 6 bits

		switch (opcode) {
		case 'A':
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); // Toggle LED1

			str = malloc(15);
			memcpy(str, "Toggled LED1\r\n", 15);
			*ret_len = strlen(str);
			strcpy(response, str);

			break;
		case 'I':
			/*
			 * Set IAP Flag
			 */
			HAL_PWR_EnableBkUpAccess();
			// Clears IAP Flag in RTC Backup data Register 1
			HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xDEE2);
			HAL_PWR_DisableBkUpAccess();
/*
			str = "IAP Flag Set\r\nResetting MCU\r\n";
			*ret_len = strlen(str);
			strcpy(response, str);

			server_sendMsg(connfd, response, ret_len, 0);*/

			NVIC_SystemReset();
			break;

		case 'R':
			/*str = "Resetting MCU\r\n";
			*ret_len = strlen(str);
			strcpy(response, str);

			server_sendMsg(connfd, response, ret_len, 0);*/

			NVIC_SystemReset();
			break;

		case 'G':
			*ret_len = strlen(GIT_INFO);
			strcpy(response, GIT_INFO);

			break;

		default:
			str = malloc(18);
			memcpy(str, "Unknown Command\r\n", 18);
			strcpy(response, str);

			break;
		}

		return PARSE_OK;
	} else {
		return PARSE_ERR;
	}

}
#endif

void msgFreeCallback(void * data) {
	free(data);
}

/*
 * Starts the TCP server
 * Creates a listen thread, a recv thread, and a send thread
 */
void server_init() {

	memset(connections, -1, MAX_CONN_NUM * sizeof(int));

	conn_mu = malloc(sizeof(sys_mutex_t));
	err_t err = sys_mutex_new(conn_mu);

	// size of list element is max message size + length and connfd integers
	rxMsgBuffer = list_create(MAX_MSG_LEN + 2*sizeof(int), msgFreeCallback);
	txMsgBuffer = list_create(MAX_MSG_LEN + 2*sizeof(int), msgFreeCallback);

	if (err == ERR_OK) {

		// prime server to connect to clients
		if ( NULL == sys_thread_new("server_listen_thread", server_listen, NULL, 512,
						osPriorityNormal)) {
		}

		// prime server to receive messages from clients
		if ( NULL == sys_thread_new("server_recv_thread", server_recv, NULL, 512,
						osPriorityNormal)) {
		}

		// prime server to send messages to clients
		if ( NULL == sys_thread_new("server_recv_thread", server_send, NULL, 512,
						osPriorityNormal)) {
		}
	}
	else {
		printf("Error in server init\r\n");
	}
}

/*
 * Sets the server in listen mode
 * When a connection is established, a fd is generated and pushed to the connections list for use
 */
void server_listen(void *arg) {
	int err;
	int listen_sockfd, connfd;
	socklen_t clilen;

	/* IPv4 socket address structure */
	struct sockaddr_in serv, cli;
	serv.sin_family = AF_INET;
	serv.sin_port = htons(SERVER_PORT);
	serv.sin_addr.s_addr = (in_addr_t) ipaddr.addr;

	LWIP_UNUSED_ARG(arg);

	/* Create a new socket fd for listening */
	listen_sockfd = socket(AF_INET, SOCK_STREAM, 6); // 6 is the DARPA protocol # for tcp

	if (listen_sockfd != -1) {
		/* Bind socket to server port */
		err = bind(listen_sockfd, (struct sockaddr* ) &serv, sizeof(serv));

		if (err == 0) {
			/* Tell connection to go into listening mode. */
			err = listen(listen_sockfd, MAX_CONN_NUM);

			for (;;) {
				clilen = sizeof(cli);

				// Block until new connection, accept any that appear
				connfd = accept(listen_sockfd, (struct sockaddr* ) &cli, &clilen);

				server_addConnection(connfd);
			} // end for(;;)
		} else { // Bind failed
			close(listen_sockfd);
		}
	} else { // Socket creation failed

	}
}

/*
 * Adds socket to connections list
 */
void server_addConnection(int connfd) {

	sys_mutex_lock(conn_mu);

	for (int i = 0; i < MAX_CONN_NUM; ++i) {
		if (connections[i] == -1) {
			connections[i] = connfd;

			sys_mutex_unlock(conn_mu);
			return;
		}
	}

	sys_mutex_unlock(conn_mu);

}

/*
 * Removes socket from connections list and frees fd for a new socket to use
 */
void server_removeConnection(int connfd) {

	sys_mutex_lock(conn_mu);

	for (int i = 0; i < MAX_CONN_NUM; ++i) {
		if (connections[i] == connfd) {
			connections[i] = -1;
			close(connfd);

			sys_mutex_unlock(conn_mu);
			return;
		}
	}

	sys_mutex_unlock(conn_mu);
}

void server_removeAllConnections(void) {

	sys_mutex_lock(conn_mu);

	for (int i = 0; i < MAX_CONN_NUM; ++i) {
		if (connections[i] != -1) {
			close(connections[i]);
			connections[i] = -1;
		}
	}

	sys_mutex_unlock(conn_mu);
}

void server_setFDs(fd_set *rfds) {
	FD_ZERO(rfds);

	sys_mutex_lock(conn_mu);

	for (int i = 0; i < MAX_CONN_NUM; ++i) {
		if (connections[i] != -1) {
			FD_SET(connections[i], rfds);
		}
	}

	sys_mutex_unlock(conn_mu);
}

void server_handleRecv(fd_set *rfds) {

	sys_mutex_lock(conn_mu);

	for (int i = 0; i < MAX_CONN_NUM; ++i) {

		if (connections[i] != -1) {
			int connfd = connections[i];

			if (FD_ISSET(connfd, rfds)) { // FD data recv

				char* buf = malloc(MAX_MSG_LEN);

				int n = recv(connfd, buf, MAX_MSG_LEN, 0);
				// recv all waiting data

				struct message msg = {connfd, buf, n};

				list_push(rxMsgBuffer, (void*)(&msg));

			}
		}
		sys_mutex_unlock(conn_mu);
	}
}

void server_recv(void *arg) {
	fd_set rfds;
	struct timeval tv;
	int retval;

	LWIP_UNUSED_ARG(arg);

	/* Process the new connection. */
	/* receive the data from the client */
	for (;;) {
		server_setFDs(&rfds);

		// Wait 1 second
		tv.tv_sec = 1;
		tv.tv_usec = 0;

		retval = select(MAX_CONN_NUM+1, &rfds, NULL, NULL, &tv);

		if (retval == -1) { // error
			//server_probeConnections();
		} else if (retval) { // FD_ISSET will have some true fd
			server_handleRecv(&rfds);
		} else { // timeout
			/*if (server_sendMsg(newconn, (void*)"", (size_t)1, NETCONN_COPY) != ERR_OK) {
			 // connection closed
			 break;
			 }*/
		}
	}

	server_removeAllConnections();

	vTaskDelete(NULL);
}

void server_sendMsg(int destIP, char* data, int len) {

	if (destIP == ALL_CONNECTIONS) {
		sys_mutex_lock(conn_mu);
		for (int i = 0; i < MAX_CONN_NUM; ++i) {

			if (connections[i] != -1) {
				int connfd = connections[i];

				struct message msg = {connfd, data, len};

				list_push(txMsgBuffer, (void*)(&msg));
			}
		}
		sys_mutex_unlock(conn_mu);
	} else { // actual IP specified
		sys_mutex_lock(conn_mu);
//		for (int i = 0; i < MAX_CONN_NUM; ++i) {
//
//			if (connections[i] != -1 && connections[i]) {
//				int connfd = connections[i];

				struct message msg = {destIP, data, len};

				list_push(txMsgBuffer, (void*)(&msg));
//				break;
//			}
//		}
		sys_mutex_unlock(conn_mu);
	}
}

void server_retrieveMsg(struct message* msg) {
	list_pop(rxMsgBuffer, (void*)(msg));
}

void server_send(void* args) {
	LWIP_UNUSED_ARG(args);

	char buf[MAX_MSG_LEN];

	for (;;) {

		struct message msg = {-1, buf, MAX_MSG_LEN};

		list_pop(txMsgBuffer, (void*)(&msg));

		if (-1 == send(msg.connfd, msg.buf, msg.len, 0)) { // opts = 0
			// message failed to send
			server_removeConnection(msg.connfd);
		}

		free(msg.buf);
	}

	server_removeAllConnections();

	vTaskDelete(NULL);
}
