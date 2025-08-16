/*
 * server.c
 *
 *  Created on: Feb 16, 2025
 *      Author: jackmh
 */

#include "server.h"
#include "semphr.h"
#include "errno.h"
#include "stdio.h"
#include "log_errors.h"

extern ip4_addr_t ipaddr; // get our IP address from somewhere
#define SERVER_PORT TCP_PORT
#define MAX_CONN_NUM 8

List* rxMsgBuffer = NULL; // thread-safe queue for incoming messages
List* txMsgBuffer = NULL;

int connections[MAX_CONN_NUM]; // List of active connections
sys_mutex_t conn_mu; // Mutex for the connections list

SemaphoreHandle_t shutdownStart;
SemaphoreHandle_t shutdownDone;
uint8_t running = 0;
SemaphoreHandle_t runningMutex;
int devices[5] = {-1, -1, -1, -1, -1};
SemaphoreHandle_t deviceMutex;
ip4_addr_t deviceIPs[5];
SemaphoreHandle_t closeMutex;

// initialize semaphores/mutexes
int server_create(ip4_addr_t limewire, ip4_addr_t bb1, ip4_addr_t bb2, ip4_addr_t bb3, ip4_addr_t fr) {
	deviceIPs[LimeWire_d] = limewire;
	deviceIPs[BayBoard1_d] = bb1;
	deviceIPs[BayBoard2_d] = bb2;
	deviceIPs[BayBoard3_d] = bb3;
	deviceIPs[FlightRecorder_d] = fr;

	shutdownStart = xSemaphoreCreateCounting(3, 0);
    shutdownDone = xSemaphoreCreateCounting(3, 0);
    runningMutex = xSemaphoreCreateMutex();
    deviceMutex = xSemaphoreCreateMutex();
    closeMutex = xSemaphoreCreateMutex();

    if(!shutdownStart || !shutdownDone || !runningMutex || !deviceMutex || !closeMutex) {
    	return 1;
    }

    // check if the mutex got created successfully
    if(sys_mutex_new(&conn_mu) != ERR_OK) {
        return 1;
    }

    // make thread-safe queues for incoming and outgoing messages
    rxMsgBuffer = list_create(sizeof(Raw_message), msgFreeCallback);
    if(!rxMsgBuffer) {
    	return 1;
    }
	txMsgBuffer = list_create(sizeof(Raw_message), msgFreeCallback);
    if(!txMsgBuffer) {
    	return 1;
    }
    return 0;
}


// Function to initialize the server
// Spins off a listener, reader, and writer task/thread 
// as well as initializes the message buffers and connection list
int server_init(void) {
	if(!txMsgBuffer) {
		return -3;
	}

	if(xSemaphoreTake(runningMutex, 5) == pdPASS) {
		if(running) {
			xSemaphoreGive(runningMutex);
			return -1;
		}
		else {
			running = 1;
			xSemaphoreGive(runningMutex);
		}
	}
	else {
		return -1;
	}

    osThreadId_t readerTaskHandle, writerTaskHandle, listenerTaskHandle;
    // task configuration
    const osThreadAttr_t listenerTask_attributes = {
      .name = "listenerTask",
      .stack_size = 512 * 16, // I just increased this until it worked, it may be able to be reduced
      .priority = (osPriority_t) osPriorityNormal,
    };
    const osThreadAttr_t readerTask_attributes = {
      .name = "readerTask",
      .stack_size = 512 * 6,
      .priority = (osPriority_t) osPriorityNormal,
    };
    const osThreadAttr_t writerTask_attributes = {
      .name = "writerTask",
      .stack_size = 512 * 6,
      .priority = (osPriority_t) osPriorityNormal,
    };

    // Note: this doesn't need to be a ts-queue it's just a list of 
    // which connections are active where order does not matter
    memset(connections, -1, MAX_CONN_NUM * sizeof(int)); // fill non-active connections with -1

    if(xSemaphoreTake(deviceMutex, 10) == pdPASS) {
    	memset(devices, -1, 5 * sizeof(int));
    	xSemaphoreGive(deviceMutex);
    }

    // Reset shutdownStart semaphore to 0, in case one or more of the threads exited early
    while(xSemaphoreTake(shutdownStart, 0) == pdTRUE) {}

    drain_lists();

    // start the tasks
    listenerTaskHandle = osThreadNew(server_listener_thread, NULL, &listenerTask_attributes);
    readerTaskHandle = osThreadNew(server_reader_thread, NULL, &readerTask_attributes);
    writerTaskHandle = osThreadNew(server_writer_thread, NULL, &writerTask_attributes);

    // check if the tasks got created successfully
    if ((listenerTaskHandle == NULL) ||
        (readerTaskHandle == NULL) ||
        (writerTaskHandle == NULL)) {
        return -2;
    }
    return 0;
}

// Listen for incoming connections and add them to the connections list
void server_listener_thread(void *arg) {
    LWIP_UNUSED_ARG(arg);

    // Create new socket for listening. 6 is the protocol # for tcp
    int listen_sockfd = -1;
    for(;;) {
    	listen_sockfd = socket(AF_INET, SOCK_STREAM, 6);
        if(listen_sockfd == -1) {
        	switch(errno) {
        	    case ENOBUFS: {
        	        // no memory - couldn't create netconn
        	    	log_message(FC_ERR_TCP_SERV_SOCK_CREAT_NOBUF, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case ENFILE: {
        	        // couldn't create socket, none available
        	    	log_message(FC_ERR_TCP_SERV_SOCK_CREAT_NOSOCK, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    default: {
        	    	// unknown error when creating socket, use errno in message
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_SOCK_CREAT_UNKNOWN) + 3];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_SOCK_CREAT_UNKNOWN "%d", errno);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	}
            osDelay(TCP_RETRY_DELAY_MS);
            continue;
        }

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        setsockopt(listen_sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        // Bind socket to port
        struct sockaddr_in server_socket;
        server_socket.sin_family = AF_INET; // IPv4
        server_socket.sin_port = htons(SERVER_PORT); // grab server port
        server_socket.sin_addr.s_addr = (in_addr_t) ipaddr.addr; // get IP from extern

        if(bind(listen_sockfd, (struct sockaddr *) &server_socket, sizeof(server_socket)) != 0) {
        	switch(errno) {
        	    case EBADF: {
        	        // bad socket, include the fact that the listener thread is stopping
        	    	log_message(FC_ERR_TCP_SERV_SOCK_BIND_NSOCK, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case EINVAL: {
        	        // pcb not closed or NULL pcb, include the fact that the listener thread is stopping
        	    	log_message(FC_ERR_TCP_SERV_SOCK_BIND_BADPCB, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case EADDRINUSE: {
        	        // address already in use, there is already a socket listening on the port, include the fact that the listener thread is stopping
        	    	log_message(FC_ERR_TCP_SERV_SOCK_BIND_USEDADDR, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    default: {
        	    	// unknown error when binding socket, use errno in message, include the fact that the listener thread is stopping
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_SOCK_BIND_UNKNOWN) + 3];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_SOCK_BIND_UNKNOWN "%d", errno);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	}
            close(listen_sockfd);
            osDelay(TCP_RETRY_DELAY_MS);
            continue;
        }

        // Listen for incoming connections (blocks until a connection is made)
        if(listen(listen_sockfd, MAX_CONN_NUM) < 0) {
        	switch(errno) {
        	    case EBADF: {
        	        // bad socket, include the fact that the listener thread is stopping
        	    	log_message(FC_ERR_TCP_SERV_SOCK_LISTEN_NSOCK, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case EIO: {
        	        // NULL netconn, include the fact that the listener thread is stopping
        	    	log_message(FC_ERR_TCP_SERV_SOCK_LISTEN_NNETCONN, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case ENOTCONN: {
        	        // NULL pcb or netconn already started and not in listening mode, include the fact that the listener thread is stopping
        	    	log_message(FC_ERR_TCP_SERV_SOCK_LISTEN_NPCB, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case EINVAL: {
        	        // pcb not closed, include the fact that the listener thread is stopping
        	    	log_message(FC_ERR_TCP_SERV_SOCK_LISTEN_BADPCB, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case ENOMEM: {
        	        // out of memory when creating the pcb listener, include the fact that the listener thread is stopping
        	    	log_message(FC_ERR_TCP_SERV_SOCK_LISTEN_NOMEM, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    default: {
        	    	// unknown error when setting socket to listening mode, use errno in message, include the fact that the listener thread is stopping
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_SOCK_LISTEN_UNKNOWN) + 3];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_SOCK_LISTEN_UNKNOWN "%d", errno);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	}
            close(listen_sockfd);
            osDelay(TCP_RETRY_DELAY_MS);
            continue;
        }
        break;
    }

    for (;;) {
    	if(xSemaphoreTake(shutdownStart, 0) == pdPASS) {
    		// Shutdown
    		break;
    	}
        // we have a connection!
        struct sockaddr_in connection_socket;
        socklen_t addr_len = sizeof(connection_socket);

        int connection_fd = accept(listen_sockfd, (struct sockaddr* ) &connection_socket, &addr_len);
        if(connection_fd < 0) {
        	// Socket timeout or close/error
        	// None of these error cases will close the thread, since even though a couple should, it's good to spam the message so the COP can address it quickly
        	// This is different from the listener thread startup errors, since that happens once, and the COP should be paying attention during startup
        	// In the future possibly add support to retry the startup steps if something fails
        	switch(errno) {
    	    	case EWOULDBLOCK: {
    	    		// This is expected and SHOULD happen, so don't log
    	    		break;
    	    	}
        	    case EBADF: {
        	        // bad socket, probably the server shutting down
        	    	log_message(FC_ERR_TCP_SERV_SOCK_ACCEPT_NSOCK, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case EIO: {
        	        // listening netconn is NULL
        	    	log_message(FC_ERR_TCP_SERV_SOCK_ACCEPT_NNETCONN, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case ENFILE: {
        	        // no socket available
        	    	log_message(FC_ERR_TCP_SERV_SOCK_ACCEPT_NO_SOCK, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case ENOMEM: {
        	        // listening netconn out of memory
        	    	log_message(FC_ERR_TCP_SERV_SOCK_ACCEPT_NOMEM, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case ENOBUFS: {
        	        // listening netconn buffer error (could be out of memory, could be other)
        	    	log_message(FC_ERR_TCP_SERV_SOCK_ACCEPT_BUF_ERR, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case EINVAL: {
        	        // listening socket closed, probably the server shutting down
        	    	log_message(FC_ERR_TCP_SERV_SOCK_ACCEPT_LSOCK_CLSD, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case ECONNABORTED: {
        	        // listening socket aborted internally, could be out of netconns or pcbs
        	    	log_message(FC_ERR_TCP_SERV_SOCK_ACCEPT_LSOCK_ERR, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    case ENOTCONN: {
        	        // error getting connected socket info
        	    	log_message(FC_ERR_TCP_SERV_SOCK_ACCEPT_ASOCK_ERR, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	    default: {
        	    	// unknown error when accepting connection, use errno in message
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_SOCK_ACCEPT_UNKNOWN) + 3];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_SOCK_ACCEPT_UNKNOWN "%d", errno);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_LISTEN);
        	        break;
        	    }
        	}
        	continue;
        }

        int idle = TCP_KEEP_ALIVE_IDLE;
        int intvl = TCP_KEEP_ALIVE_INTERVAL;
        int optval = 1;
        int probecnt = TCP_KEEP_ALIVE_COUNT;
        struct timeval conntimeout;
        conntimeout.tv_sec = 0;
        conntimeout.tv_usec = 100000;
        setsockopt(connection_fd, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval));
        setsockopt(connection_fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(connection_fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(connection_fd, IPPROTO_TCP, TCP_KEEPCNT, &probecnt, sizeof(probecnt));
        setsockopt(connection_fd, SOL_SOCKET, SO_RCVTIMEO, &conntimeout, sizeof(conntimeout));
        setsockopt(connection_fd, SOL_SOCKET, SO_SNDTIMEO, &conntimeout, sizeof(conntimeout));

        // add the new connection to the active connections list
        sys_mutex_lock(&conn_mu);
	    for (int i = 0; i < MAX_CONN_NUM; ++i) {
	    	if (connections[i] == -1) { // -1 is an empty space in the list
	    		connections[i] = connection_fd;
                break;
	    	}
	    }
	    sys_mutex_unlock(&conn_mu);

	    for(int i = 0; i < 5; i++) {
	    	if(deviceIPs[i].addr == (u32_t) connection_socket.sin_addr.s_addr) {
	    	    if(xSemaphoreTake(deviceMutex, 10) == pdPASS) {
	    	    	devices[i] = connection_fd;
	    	    	xSemaphoreGive(deviceMutex);
	    	    	break;
	    	    }
	    	}
	    }
	    // log new connection, include given fd and ip address
    	char logmsg[sizeof(FC_STAT_TCP_SERV_NEW_CONN) + 22];
    	ip4_addr_t incomingaddr;
    	inet_addr_to_ip4addr(&incomingaddr, &connection_socket.sin_addr);
    	snprintf(logmsg, sizeof(logmsg), FC_STAT_TCP_SERV_NEW_CONN "%d/%u.%u.%u.%u", (int16_t) connection_fd, ip4_addr1(&incomingaddr), ip4_addr2(&incomingaddr), ip4_addr3(&incomingaddr), ip4_addr4(&incomingaddr));
    	log_message(logmsg, -1);
    }
    // Log exiting listener thread due to a shutdown
    log_message(FC_ERR_TCP_SERV_LISTEN_THREAD_CLOSE, -1);
    close(listen_sockfd);
    xSemaphoreGive(shutdownDone);
	vTaskDelete(NULL);
}

// Continually reads from the RX Message Buffer and processes new messages
// Also removes them from the buffer after processing
void server_reader_thread(void *arg) {
    LWIP_UNUSED_ARG(arg);

    // In order to capture incoming message we need an fd_set
    // which contains the active connections and whether or not they have incoming messages
    // but is represented as a bitmask instead of actual file descriptors
    // this is so that the `select()` function can efficiently check for incoming messages
    fd_set rx_fd_set;
    fd_set err_fd_set;
    struct timeval timeout; // timeout for select()
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100ms timeout

    for (;;) {
    	if(xSemaphoreTake(shutdownStart, 0) == pdPASS) {
    		// Shutdown
    		break;
    	}

        // capture incoming messages

        // update our fd_set with actual active connections
        // (we do this each loop in case we add or remove connections)
        int maxfd = update_fd_set(&rx_fd_set, &err_fd_set);

        // check for incoming messages
        // select is a blocking function that will return when there is an incoming message
        int nready = select(maxfd + 1, &rx_fd_set, NULL, &err_fd_set, &timeout);
        if(nready > 0) {
            // we have incoming messages

            // lock the connections mutex
            sys_mutex_lock(&conn_mu);

            // go through our connections to see which one this maps to
            for (int i = 0; i < MAX_CONN_NUM; i++) {
                if (connections[i] == -1) { // only consider active connections
                    continue;
                }

                int connection_fd = connections[i];

                if (FD_ISSET(connection_fd, &rx_fd_set)) {
                    // make space for the packet
                	uint8_t tempbuffer[MAX_MSG_LEN];

                    // read the message
				    int packet_len = recv(connection_fd, tempbuffer, MAX_MSG_LEN, 0);

				    if(packet_len == 0) {
				    	// Connection is closed
				    	xSemaphoreTake(closeMutex, portMAX_DELAY);
				    	close(connection_fd);
				    	connections[i] = -1;

			    	    if(xSemaphoreTake(deviceMutex, 10) == pdPASS) {
					    	for(int j = 0; j < 5;j++) {
					    		if(connection_fd == devices[j]) {
					    			devices[j] = -1;
					    			break;
					    		}
					    	}
			    	    	xSemaphoreGive(deviceMutex);
			    	    }
			    	    xSemaphoreGive(closeMutex);
			    	    // log connection closed, include fd
	        	    	char logmsg[sizeof(FC_STAT_TCP_CONN_CLOSED) + 6];
	        	    	snprintf(logmsg, sizeof(logmsg), FC_STAT_TCP_CONN_CLOSED "%d", (int16_t) connection_fd);
	        	    	log_message(logmsg, -1);
				    	continue;
				    }
				    else if(packet_len == -1) {
				    	switch(errno) {
				    	    case EBADF: {
				    	        // invalid socket, closing and cleaning up fd
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_READ_NSOCK) + 6];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_READ_NSOCK "%d", (int16_t) connection_fd);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
				    	        break;
				    	    }
				    	    case EWOULDBLOCK: {
				    	        // receive timeout, not very critical, just try again
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_READ_TIMEOUT) + 6];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_READ_TIMEOUT "%d", (int16_t) connection_fd);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
				    	        break;
				    	    }
				    	    case ECONNRESET: {
				    	        // connection reset by peer, closing and cleaning up fd
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_READ_CONN_RST) + 6];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_READ_CONN_RST "%d", (int16_t) connection_fd);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
				    	        break;
				    	    }
				    	    case ENOTCONN: {
				    	        // socket is not connected, closing and cleaning up fd
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_READ_SOCK_NCONN) + 6];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_READ_SOCK_NCONN "%d", (int16_t) connection_fd);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
				    	        break;
				    	    }
				    	    case ENOMEM: {
				    	        // out of memory when receiving data
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_READ_NOMEM) + 6];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_READ_NOMEM "%d", (int16_t) connection_fd);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
				    	        break;
				    	    }
				    	    case ENOBUFS: {
				    	        // out of buffer space
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_READ_NOBUF) + 6];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_READ_NOBUF "%d", (int16_t) connection_fd);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
				    	        break;
				    	    }
				    	    default: {
				    	    	// unknown error when receiving data, use errno in message
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_READ_UNKNOWN) + 10];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_READ_UNKNOWN "%d/%d", (int16_t) connection_fd, errno);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
				    	        break;
				    	    }
				    	}
				    	if(errno == EBADF || errno == ECONNRESET || errno == ENOTCONN) {
					    	xSemaphoreTake(closeMutex, portMAX_DELAY);
					    	close(connection_fd);
					    	connections[i] = -1;

				    	    if(xSemaphoreTake(deviceMutex, 10) == pdPASS) {
						    	for(int j = 0; j < 5;j++) {
						    		if(connection_fd == devices[j]) {
						    			devices[j] = -1;
						    			break;
						    		}
						    	}
				    	    	xSemaphoreGive(deviceMutex);
				    	    }
				    	    xSemaphoreGive(closeMutex);
				    	}
				    	continue;
				    }

				    Raw_message msg = {0};
				    uint8_t *buffer = malloc(packet_len);
				    if(buffer) {
					    memcpy(buffer, tempbuffer, packet_len);
					    msg.bufferptr = buffer;
					    msg.connection_fd = connection_fd;
					    msg.packet_len = packet_len;

					    // add the message to the RX Message Buffer to be parsed
	                    if(list_push(rxMsgBuffer, (void*)(&msg), portMAX_DELAY)) {
	                    	// JUST in case
	                    	free(buffer);
	                    }
				    }
				    else {
				    	// no memory creating receiving buffer
	        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_STORE_NOMEM) + 6];
	        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_STORE_NOMEM "%d", (int16_t) connection_fd);
	        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
				    }
                }
                else if(FD_ISSET(connection_fd, &err_fd_set)) {
                    int err = 0;
                    socklen_t len = sizeof(err);
                    if(getsockopt(connection_fd, SOL_SOCKET, SO_ERROR, &err, &len) == 0 && err != 0) {
                        switch(err) {
                        	case ECONNRESET: {
                        		// reset by peer, closing
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_ERROR_RESET) + 6];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_ERROR_RESET "%d", (int16_t) connection_fd);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
                        		break;
                        	}
                        	case ETIMEDOUT: {
                        		// unresponsive peer
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_ERROR_TIMEO) + 6];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_ERROR_TIMEO "%d", (int16_t) connection_fd);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
                        		break;
                        	}
                        	case EPIPE: {
                        		// connection closed
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_ERROR_CLOSED) + 6];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_ERROR_CLOSED "%d", (int16_t) connection_fd);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
                        		break;
                        	}
                        	case ENOTCONN: {
                        		// socket not connected
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_ERROR_NOTCONN) + 6];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_ERROR_NOTCONN "%d", (int16_t) connection_fd);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
                        		break;
                        	}
                        	default: {
                        		// unknown
			        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_ERROR_UNKNOWN) + 10];
			        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_ERROR_UNKNOWN "%d/%d", (int16_t) connection_fd, err);
			        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_READ);
                        		break;
                        	}
                        }
                        if(err == ECONNRESET || err == ETIMEDOUT || err == EPIPE || err == ENOTCONN) {
    					    xSemaphoreTake(closeMutex, portMAX_DELAY);
    					    close(connection_fd);
    					    connections[i] = -1;

    				    	if(xSemaphoreTake(deviceMutex, 10) == pdPASS) {
    						    for(int j = 0; j < 5;j++) {
    						    	if(connection_fd == devices[j]) {
    						    		devices[j] = -1;
    						    		break;
    						    	}
    						    }
    				    	    xSemaphoreGive(deviceMutex);
    				    	}
    				    	xSemaphoreGive(closeMutex);
                        }
                    }
                    continue;
                }
            }

            // unlock the connections mutex
            sys_mutex_unlock(&conn_mu);
        }
        else if(nready < 0) {
        	switch(errno) {
        	    case EBADF: {
        	        // bad socket in fd list, this could be because the server is shutting down, scanning fd list and removing closed sockets
		    	    log_message(FC_ERR_TCP_SERV_RECV_WAIT_NSOCK, FC_ERR_TYPE_TCP_SERV_RECV_SELECT);
        	    	remove_bad_fds();
        	        break;
        	    }
        	    case ENOMEM: {
        	        // out of memory creating semaphores to signal ready sockets
		    	    log_message(FC_ERR_TCP_SERV_RECV_WAIT_NOMEM, FC_ERR_TYPE_TCP_SERV_RECV_SELECT);
        	        break;
        	    }
        	    case EBUSY: {
        	        // too many threads waiting on one or more of the sockets
		    	    log_message(FC_ERR_TCP_SERV_RECV_WAIT_BUSY_SOCK, FC_ERR_TYPE_TCP_SERV_RECV_SELECT);
        	        break;
        	    }
        	    default: {
        	    	// unknown error when waiting for socket events
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_RECV_WAIT_UNKNOWN) + 3];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_RECV_WAIT_UNKNOWN "%d", errno);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_RECV_SELECT);
        	        break;
        	    }
        	}
        }
    }
    // Log exiting receive thread due to a shutdown
    log_message(FC_ERR_TCP_SERV_RECV_THREAD_CLOSE, -1);
    xSemaphoreGive(shutdownDone);
	vTaskDelete(NULL);
}

// Continually checks for new messages in the TX Message Buffer and sends them to the client
// Also removes them from the buffer after sending
void server_writer_thread(void *arg) {
	LWIP_UNUSED_ARG(arg);

	for (;;) {
    	if(xSemaphoreTake(shutdownStart, 0) == pdPASS) {
    		// Shutdown
    		break;
    	}
		Raw_message msg = {0};

        // blocks until a message is available in the TX Message Buffer
		if(list_pop(txMsgBuffer, (void*)(&msg), 100)) {
			// returned early, no messages
			continue;
		}

        if (msg.connection_fd == -1) {
            // messages with -1 connfd are not valid
        	free(msg.bufferptr);
            continue;
        }
        xSemaphoreTake(closeMutex, portMAX_DELAY);
		if(send(msg.connection_fd, msg.bufferptr, msg.packet_len, 0) == -1) { // opts = 0
	    	switch(errno) {
	    	    case EBADF: {
	    	        // invalid socket, letting receive task clean up the connection
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_WRITE_SEND_NSOCK) + 6];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_WRITE_SEND_NSOCK "%d", (int16_t) msg.connection_fd);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_WRITE);
	    	        break;
	    	    }
	    	    case EINPROGRESS: {
	    	        // internal netconn is connecting, closing, or writing in a different place
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_WRITE_SEND_SOCK_BUSY) + 6];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_WRITE_SEND_SOCK_BUSY "%d", (int16_t) msg.connection_fd);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_WRITE);
	    	        break;
	    	    }
	    	    case ENOTCONN: {
	    	        // invalid netconn pcb
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_WRITE_SEND_NPCB) + 6];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_WRITE_SEND_NPCB "%d", (int16_t) msg.connection_fd);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_WRITE);
	    	        break;
	    	    }
	    	    case EIO: {
	    	        // NULL netconn, this should be caught in EBADF but in a rare case this might happen
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_WRITE_SEND_NNETCONN) + 6];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_WRITE_SEND_NNETCONN "%d", (int16_t) msg.connection_fd);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_WRITE);
	    	        break;
	    	    }
	    	    case EINVAL: {
	    	        // internal overflow or NULL pointer
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_WRITE_SEND_MEM_ERR) + 6];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_WRITE_SEND_MEM_ERR "%d", (int16_t) msg.connection_fd);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_WRITE);
	    	        break;
	    	    }
	    	    default: {
	    	    	// unknown error when sending data, use errno in message
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_WRITE_SEND_UNKNOWN) + 10];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_WRITE_SEND_UNKNOWN "%d/%d", (int16_t) msg.connection_fd, errno);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_SERV_WRITE);
	    	        break;
	    	    }
	    	}
		}
		xSemaphoreGive(closeMutex);
		free(msg.bufferptr);

	}
    // Log exiting writer thread due to a shutdown
	log_message(FC_ERR_TCP_SERV_WRITE_THREAD_CLOSE, -1);
    xSemaphoreGive(shutdownDone);
	vTaskDelete(NULL);
}

// Callback function to free message data
// TBH I don't get why this is needed but it's for the TS-queue
void msgFreeCallback(void * data) {
	free(data);
}

int server_read(Raw_message* msg, TickType_t block) {
    // Pop a message from the RX Message Buffer
    // This will block until a message is available
	if(is_server_running() > 0) {
		if(list_pop(rxMsgBuffer, (void*)msg, block)) {
			// No message available
			return -2;
		}

		// Return the length of the message
		return msg->packet_len;
	}
	else {
		return -1;
	}
}

// Helper function to add a message to the TX Message Buffer
int server_send(Raw_message* msg, TickType_t block) {
	if(is_server_running() > 0) {
		// Push a message to the TX Message Buffer
		// This will block until there is space in the buffer
		if(list_push(txMsgBuffer, (void*)msg, block)) {
			// Timed out
			return -2;
		}

		return 0; // success
	}
	else {
		return -1;
	}
}

// compares the connections list to the fd_set and sets the fd_set to the active connections, also returns the max fd for use in select()
int update_fd_set(fd_set *rfds, fd_set *efds) {
    FD_ZERO(rfds);
    FD_ZERO(efds);
    int max = 0;

	sys_mutex_lock(&conn_mu);

	for (int i = 0; i < MAX_CONN_NUM; ++i) {
		if(connections[i] != -1) {
			FD_SET(connections[i], rfds);
			FD_SET(connections[i], efds);
		}
		if(connections[i] > max) {
			max = connections[i];
		}
    }

	sys_mutex_unlock(&conn_mu);
    return max;
}

int is_server_running() {
	if(xSemaphoreTake(runningMutex, 2) == pdPASS) {
		uint8_t stat = running;
		xSemaphoreGive(runningMutex);
		return stat;
	}
	else {
		return -1;
	}
}

void drain_lists() {
	Raw_message msg = {0};
	while(!list_pop(txMsgBuffer, (void*)(&msg), 0)) {
		free(msg.bufferptr);
	}

	while(!list_pop(rxMsgBuffer, (void*)(&msg), 0)) {
		free(msg.bufferptr);
	}
}

int shutdown_server() {
	if(xSemaphoreTake(runningMutex, portMAX_DELAY) == pdPASS) {
		if(!running) {
			xSemaphoreGive(runningMutex);
			return -1;
		}
		else {
			running = 0;
			xSemaphoreGive(runningMutex);
		}

	    if(xSemaphoreTake(deviceMutex, 10) == pdPASS) {
	    	memset(devices, -1, 5 * sizeof(int));
	    	xSemaphoreGive(deviceMutex);
	    }

		for(int i = 0;i < 3;i++) {
			xSemaphoreGive(shutdownStart);
		}

		for(int i = 0;i < 3;i++) {
			xSemaphoreTake(shutdownDone, portMAX_DELAY);
		}

	    for (int i = 0; i < MAX_CONN_NUM; ++i) {
	    	if(connections[i] != -1) {
	    		close(connections[i]);
	    		connections[i] = -1;
	    	}
	    }

	    // Get rid of stale messages
	    drain_lists();

	    return 0;
	}
	else {
		return -1;
	}
}

int get_device_fd(Target_Device dev) {
	if(dev >= NUM_TARGET_DEVICES) {
		return -2;
	}
    if(xSemaphoreTake(deviceMutex, portMAX_DELAY) == pdPASS) {
    	int fd = devices[dev];
    	xSemaphoreGive(deviceMutex);
    	return fd;
    }
    return -2;
}

// Scans connections for bad sockets - sockets that have been closed without setting the corresponding fd to -1
void remove_bad_fds(void) {
	sys_mutex_lock(&conn_mu);
	for(int i = 0; i < MAX_CONN_NUM; ++i) {
		if(connections[i] != -1) {
			int type;
			socklen_t len = sizeof(type);
			if(getsockopt(connections[i], SOL_SOCKET, SO_TYPE, &type, &len) == -1) {
			    if(errno == EBADF) {
			        // bad fd in list, include fd number
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_FD_SCAN_NSOCK) + 6];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_FD_SCAN_NSOCK "%d", (int16_t) connections[i]);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_FD_SCAN);

		    	    if(xSemaphoreTake(deviceMutex, 5) == pdPASS) {
				    	for(int j = 0; j < 5;j++) {
				    		if(connections[i] == devices[j]) {
				    			devices[j] = -1;
				    			break;
				    		}
				    	}
		    	    	xSemaphoreGive(deviceMutex);
		    	    }
			    	connections[i] = -1;
			    } else {
			        // unknown socket error during fd list scan, include fd number and errno number
        	    	char logmsg[sizeof(FC_ERR_TCP_SERV_FD_SCAN_UNKNOWN) + 10];
        	    	snprintf(logmsg, sizeof(logmsg), FC_ERR_TCP_SERV_FD_SCAN_UNKNOWN "%d/%d", (int16_t) connections[i], errno);
        	    	log_message(logmsg, FC_ERR_TYPE_TCP_FD_SCAN);
			    }
			}
		}
    }
	sys_mutex_unlock(&conn_mu);
}
