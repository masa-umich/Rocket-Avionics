/*
 * tcpserver.c
 *
 *  Created on: Jan 16, 2024
 *      Author: evanm
 */
#include <tcpserver.h>

typedef enum {
	PARSE_OK, PARSE_ERR,
} parse_t;

extern RTC_HandleTypeDef hrtc;

#ifndef DEVPARSE
#define DEVPARSE
parse_t devparse(char *data, u16_t len, char *response, u16_t *ret_len, struct netconn* newconn) {
	/*
	 *  Data stores len number of bytes
	 *
	 *  Packet structure:
	 *  	6 bits: opcode
	 */

	char* str;

	if (len > 0) {

		u8_t opcode = (*data );//>> 2); // get opcode from first 6 bits

		switch (opcode) {
		case 'A':
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); // Toggle LED1

			str = "Toggled LED1\r\n";
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

			str = "IAP Flag Set\r\nResetting MCU\r\n";
			*ret_len = strlen(str);
			strcpy(response, str);

			netconn_write(newconn, (void*)response, (size_t)*ret_len, NETCONN_COPY);

			NVIC_SystemReset();
			break;

		case 'R':
			str = "Resetting MCU\r\n";
			*ret_len = strlen(str);
			strcpy(response, str);

			netconn_write(newconn, (void*)response, (size_t)*ret_len, NETCONN_COPY);

			NVIC_SystemReset();
			break;

		case 'G':
			*ret_len = strlen(GIT_INFO);
			strcpy(response, GIT_INFO);

			break;

		default:
			str = "Unknown Command\r\n";
			*ret_len = strlen(str);
			strcpy(response, str);

			break;
		}

		return PARSE_OK;
	} else {
		return PARSE_ERR;
	}

}
#endif

/*
 * Starts the TCP server
 */
void server_init() {


	// prime server to connect to clients
	if ( NULL == sys_thread_new("server_connection_thread", server_waitForClientConnection, NULL,
			512, osPriorityNormal)) {

	}

	if ( NULL == sys_thread_new("server_broadcast_thread", server_broadcast, NULL,
				512, osPriorityNormal)) {

		}
}

/*
 * Sets the server in connection mode
 * When a connection is established, an ID is generated and pushed to the connections list for use
 * Server reenters connection mode
 */
static void server_waitForClientConnection(void *arg) {
	err_t err;

	LWIP_UNUSED_ARG(arg);

	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);

	if (conn != NULL) {
		/* Bind connection to ephemeral port number 50000. */
		err = netconn_bind(conn, IP4_ADDR_ANY, SERVER_PORT);

		if (err == ERR_OK) {
			/* Tell connection to go into listening mode. */
			netconn_listen(conn);

			while (1) {
				int8_t ID = server_addConnection();

				if (ID != -1) {
					// connection accepted, spawn recv thread
					char thread_name[22];
					snprintf(thread_name, 22, "server_recv_thread_%02d", ID);
					netconn_write(connections[ID], (void*)thread_name, (size_t)22, NETCONN_COPY);
					netconn_write(connections[ID], (void*)"\r\n", (size_t)2, NETCONN_COPY);

					if ( NULL == sys_thread_new(thread_name, server_recv, &ID,
							512, osPriorityNormal) ) {
						// Failed to instantiate, free socket
						server_removeConnection(ID);
					}

				} // end if
			} // end while
		}
	}
}

/*
 * Adds netconn to connections list and assigns a free ID to a netconn
 * RETURNS assigned ID, -1 for a rejected connection
 */
static int8_t server_addConnection() {
	/* Grab new connection. */
	netconn* newconn = malloc(sizeof(netconn*));

	err_t err = netconn_accept(conn, &newconn);

	if (err == ERR_OK) {
		for (int id = 0; id < MAX_CONN_NUM; ++id) {
			if (connections[id] == NULL) {
				// free ID found, assign to newconn
				connections[id] = newconn;

				 newconn->recv_timeout = 3000; // 3 sec timeout

				return id;
			}
		}
	}

	// Out of space in connection array , reject connection
	/* Close connection and discard connection identifier. */
	netconn_close(newconn);
	netconn_delete(newconn);

	free(newconn);

	return -1;
}

/*
 * Removes netconn from connections list and frees ID for a new netconn to use
 */
static void server_removeConnection(u8_t ID) {
	netconn* newconn = connections[ID];

	netconn_close(newconn);
	netconn_delete(newconn);

	free(newconn);

	connections[ID] = NULL;
}

static void server_recv(void *arg) {
	err_t err;
	struct netbuf *buf;

	char msg[MAX_MSG_LEN];
	char smsg[MAX_MSG_LEN];

	u8_t ID = ((int*)arg)[0];
	netconn* newconn = connections[ID];

	/* Process the new connection. */
	/* receive the data from the client */
	for(;;) {
		if (netconn_recv(newconn, &buf) == ERR_OK) {
			/* Extract the address and port in case they are required */
			do {
					strncpy (msg, buf->p->payload, buf->p->len);   // get the message from the client
					u16_t ret_len = 0;

					parse_t rc = devparse(msg, buf->p->len, smsg, &ret_len, newconn);

					if (ret_len > 0) {
						netconn_write(newconn, (void*)smsg, (size_t)ret_len, NETCONN_COPY);
					}

					memset (msg, '\0', 100);  // clear the buffer
				} while (netbuf_next(buf) >= 0);

			netbuf_delete(buf);
		}
		// recv timeout or error
		if (netconn_write(newconn, (void*)"", (size_t)1, NETCONN_COPY) != ERR_OK) {
			// connection closed
			break;
		}
	}

	server_removeConnection(ID);

	vTaskDelete(NULL);
}

static void server_broadcast(void *arg) {
	for (;;) {
		for (int i = 0; i < MAX_CONN_NUM; ++i) {
			if (connections[i] != NULL) {
				netconn_write(connections[i], (void*)"data\r\n", (size_t)7, NETCONN_COPY);
			}
		}
		osDelay(1000);
	}
}
/*
// Send a message to a specific client
void server_messageClient(std::shared_ptr<connection<T>> client, const message<T>& msg) {
	// Check client is legitimate
	if (client && client->IsConnected()) {

		client->Send(msg);
	} else {

		// Remove disconnected client
		OnClientDisconnect(client);
		client.reset();

		// Physically remove it from the container
		m_deqConnections.erase(
			std::remove(m_deqConnections.begin(), m_deqConnections.end(), client), m_deqConnections.end());
	}
}

// Send message to all clients
void server_messageAllClients(const message<T>& msg, std::shared_ptr<connection<T>> pIgnoreClient = nullptr) {
	bool bInvalidClientExists = false;

	// Iterate through all clients in container
	for (auto& client : m_deqConnections) {
		// Check client is legitimate
		if (client && client->IsConnected()) {

			if (client != pIgnoreClient)
				client->Send(msg);
		} else {
			// Remove disconnected client
			OnClientDisconnect(client);
			client.reset();

			// Set this flag to remove dead client from container
			bInvalidClientExists = true;
		}
	}

	// Remove dead clients, all at once to not invalidate the
	// container as we iterate through it
	if (bInvalidClientExists)
		m_deqConnections.erase(
			std::remove(m_deqConnections.begin(), m_deqConnections.end(), nullptr), m_deqConnections.end());
}

// Force server to respond to incoming messages
// -1 sets to max value since size_t is unsigned
void server_update(size_t nMaxMessages = -1, bool bWait = false) {
	if (bWait) m_qMessagesIn.wait();

	// Process as many messages as you can up to the value specified
	size_t nMessageCount = 0;
	while (nMessageCount < nMaxMessages && !m_qMessagesIn.empty()) {
		// Grab the front message
		auto msg = m_qMessagesIn.pop_front();

		// Pass to message handler
		OnMessage(msg.remote, msg.msg);

		nMessageCount++;
	}
}

// Called when a client connects, you can veto the connection by returning false
bool server_onClientConnect(std::shared_ptr<connection<T>> client) {
	return false;
}

// Called when a client appears to have disconnected
virtual void OnClientDisconnect(std::shared_ptr<connection<T>> client) {

}

// Called when a message arrives
virtual void OnMessage(std::shared_ptr<connection<T>> client, message<T>& msg) {

}

// Thread Safe Queue for incoming message packets
tsqueue<owned_message<T>> m_qMessagesIn;

// Container of active validated connections
std::deque<std::shared_ptr<connection<T>>> m_deqConnections;

// Order of declaration is important - it is also the order of initialisation
asio::io_context m_asioContext;
std::thread m_threadContext;

// These things need an asio context
tcp::acceptor m_asioAcceptor; // Handles new incoming connection attempts

// Clients will be identified in the "wider system" via an ID
uint32_t nIDCounter = 10000;
*/
