/*
 * server.c
 *
 *  Created on: Feb 16, 2025
 *      Author: jackmh
 */

#include "server.h"
#include "semphr.h"
#include "errno.h"

extern ip4_addr_t ipaddr; // get our IP address from somewhere (defined in the IOC)
#define SERVER_PORT 5000
#define MAX_CONN_NUM 8

List* rxMsgBuffer; // thread-safe queue for incoming messages
List* txMsgBuffer; 

int connections[MAX_CONN_NUM]; // List of active connections
sys_mutex_t* conn_mu; // Mutex for the connections list

SemaphoreHandle_t shutdownStart;
SemaphoreHandle_t shutdownDone;
uint8_t running = 0;
SemaphoreHandle_t runningMutex;
int devices[5] = {-1, -1, -1, -1, -1};
SemaphoreHandle_t deviceMutex;
ip4_addr_t deviceIPs[5];

// initialize semaphores/mutexes
int server_create(ip4_addr_t limewire, ip4_addr_t bb1, ip4_addr_t bb2, ip4_addr_t bb3, ip4_addr_t fr) {
    shutdownStart = xSemaphoreCreateCounting(3, 0);
    shutdownDone = xSemaphoreCreateCounting(3, 0);
    runningMutex = xSemaphoreCreateMutex();
    deviceMutex = xSemaphoreCreateMutex();

    // make thread-safe queues for incoming and outgoing messages
    rxMsgBuffer = list_create(sizeof(Raw_message), msgFreeCallback);
	txMsgBuffer = list_create(sizeof(Raw_message), msgFreeCallback);

	deviceIPs[LimeWire_d] = limewire;
	deviceIPs[BayBoard1_d] = bb1;
	deviceIPs[BayBoard2_d] = bb2;
	deviceIPs[BayBoard3_d] = bb3;
	deviceIPs[FlightRecorder_d] = fr;

	conn_mu = malloc(sizeof(sys_mutex_t));

    // check if the mutex got created successfully
    if (sys_mutex_new(conn_mu) != ERR_OK) {
        return 1;
    } else {
        return 0;
    }
}


// Function to initialize the server
// Spins off a listener, reader, and writer task/thread 
// as well as initializes the message buffers and connection list
int server_init(void) {
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

    // start the tasks
    listenerTaskHandle = osThreadNew(server_listener_thread, NULL, &listenerTask_attributes);
    readerTaskHandle = osThreadNew(server_reader_thread, NULL, &readerTask_attributes);
    writerTaskHandle = osThreadNew(server_writer_thread, NULL, &writerTask_attributes);

    // check if the tasks got created successfully
    if ((listenerTaskHandle == NULL) ||
        (readerTaskHandle == NULL) ||
        (writerTaskHandle == NULL)) {
        return 1;
    }
    return 0;
}

// Listen for incoming connections and add them to the connections list
void server_listener_thread(void *arg) {
    LWIP_UNUSED_ARG(arg);

    // Create new socket for listening. 6 is the protocol # for tcp
    int listen_sockfd = socket(AF_INET, SOCK_STREAM, 6);
    if(listen_sockfd == -1) {
    	switch(errno) {
    	    case ENOBUFS:
    	        // TODO no memory - couldn't create netconn
    	        break;
    	    case ENFILE:
    	        // TODO couldn't create socket, none available
    	        break;
    	    default:
    	    	// TODO unknown error when creating socket, use errno in message
    	        break;
    	}
        close(listen_sockfd);
        xSemaphoreGive(shutdownDone);
    	vTaskDelete(NULL);
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
    	    case EBADF:
    	        // TODO bad socket, include the fact that the listener thread is stopping
    	        break;
    	    case EIO:
    	        // TODO invalid pcb, include the fact that the listener thread is stopping
    	        break;
    	    case EINVAL:
    	        // TODO pcb not closed or NULL pcb, include the fact that the listener thread is stopping
    	        break;
    	    case EADDRINUSE:
    	        // TODO address already in use, there is already a socket listening on the port, include the fact that the listener thread is stopping
    	        break;
    	    default:
    	    	// TODO unknown error when binding socket, use errno in message, include the fact that the listener thread is stopping
    	        break;
    	}
        close(listen_sockfd);
        xSemaphoreGive(shutdownDone);
    	vTaskDelete(NULL);
    }

    // Listen for incoming connections (blocks until a connection is made)
    if(listen(listen_sockfd, MAX_CONN_NUM) < 0) {
    	switch(errno) {
    	    case EBADF:
    	        // TODO bad socket, include the fact that the listener thread is stopping
    	        break;
    	    case EIO:
    	        // TODO NULL netconn, include the fact that the listener thread is stopping
    	        break;
    	    case ENOTCONN:
    	        // TODO NULL pcb or netconn already started and not in listening mode, include the fact that the listener thread is stopping
    	        break;
    	    case EINVAL:
    	        // TODO pcb not closed, include the fact that the listener thread is stopping
    	        break;
    	    case ENOMEM:
    	        // TODO out of memory when creating the pcb listener, include the fact that the listener thread is stopping
    	        break;
    	    default:
    	    	// TODO unknown error when setting socket to listening mode, use errno in message, include the fact that the listener thread is stopping
    	        break;
    	}
        close(listen_sockfd);
        xSemaphoreGive(shutdownDone);
    	vTaskDelete(NULL);
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
        	// None of these error cases will close the thread, since even if they should, it's good to spam the message so the COP can address it quickly
        	// This is different from the listener thread startup errors, since that happens once, and the COP should be paying attention during startup
        	switch(errno) {
        	    case EBADF:
        	        // TODO bad socket, probably the server shutting down
        	        break;
        	    case EIO:
        	        // TODO listening netconn is NULL
        	        break;
        	    case ENFILE:
        	        // TODO no socket available
        	        break;
        	    case ENOMEM:
        	        // TODO listening netconn out of memory
        	        break;
        	    case ENOBUFS:
        	        // TODO listening netconn buffer error (could be out of memory, could be other)
        	        break;
        	    case EINVAL:
        	        // TODO listening socket closed, probably the server shutting down
        	        break;
        	    case ECONNABORTED:
        	        // TODO listening socket aborted internally, could be out of netconns or pcbs
        	        break;
        	    case ENOTCONN:
        	        // TODO error getting connected socket info
        	        break;
        	    default:
        	    	// TODO unknown error when accepting connection, use errno in message
        	        break;
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

        // add the new connection to the active connections list
        sys_mutex_lock(conn_mu);
	    for (int i = 0; i < MAX_CONN_NUM; ++i) {
	    	if (connections[i] == -1) { // -1 is an empty space in the list
	    		connections[i] = connection_fd;
                break;
	    	}
	    }
	    sys_mutex_unlock(conn_mu);

	    for(int i = 0; i < 5; i++) {
	    	if(deviceIPs[i].addr == (u32_t) connection_socket.sin_addr.s_addr) {
	    	    if(xSemaphoreTake(deviceMutex, 10) == pdPASS) {
	    	    	devices[i] = connection_fd;
	    	    	xSemaphoreGive(deviceMutex);
	    	    	break;
	    	    }
	    	}
	    }
    }
    // TODO Log exiting listener thread due to a shutdown, could be error or status, should have a error number tho
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
        update_fd_set(&rx_fd_set); 

        // check for incoming messages
        // select is a blocking function that will return when there is an incoming message
        int nready = select(MAX_CONN_NUM+1, &rx_fd_set, NULL, NULL, &timeout);
        if(nready > 0) {
            // we have incoming messages

            // lock the connections mutex
            sys_mutex_lock(conn_mu);

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
				    	continue;
				    }
				    else if(packet_len == -1) {
				    	switch(errno) {
				    	    case EBADF:
				    	        // TODO invalid socket, closing and cleaning up fd
				    	        break;
				    	    case EWOULDBLOCK:
				    	        // TODO receive timeout
				    	        break;
				    	    case ECONNRESET:
				    	        // TODO connection reset by peer, closing and cleaning up fd
				    	        break;
				    	    case ENOTCONN:
				    	        // TODO socket is not connected, closing and cleaning up fd
				    	        break;
				    	    case ENOMEM:
				    	        // TODO out of memory when receiving data
				    	        break;
				    	    case ENOBUFS:
				    	        // TODO out of buffer space
				    	        break;
				    	    default:
				    	    	// TODO unknown error when receiving data, use errno in message
				    	        break;
				    	}
				    	if(errno == EBADF || errno == ECONNRESET || errno == ENOTCONN) {
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
				    	}
				    	continue;
				    }

				    Raw_message msg = {0};
				    uint8_t *buffer = malloc(packet_len);
				    memcpy(buffer, tempbuffer, packet_len);
				    msg.bufferptr = buffer;
				    msg.connection_fd = connection_fd;
				    msg.packet_len = packet_len;

				    // add the message to the RX Message Buffer to be parsed
                    list_push(rxMsgBuffer, (void*)(&msg), portMAX_DELAY);
                }
            }

            // unlock the connections mutex
            sys_mutex_unlock(conn_mu);
        }
        else if(nready == -1) {
        	switch(errno) {
        	    case EBADF:
        	        // TODO bad socket in fd list, this could be because the server is shutting down, scanning fd list and removing closed sockets
        	    	remove_bad_fds();
        	        break;
        	    case ENOMEM:
        	        // TODO out of memory creating semaphores to signal ready sockets
        	        break;
        	    case EBUSY:
        	        // TODO too many threads waiting on one or more of the sockets
        	        break;
        	    default:
        	    	// TODO unknown error when waiting for socket events
        	        break;
        	}
        }
    }
    // TODO Log exiting receive thread due to a shutdown, could be error or status, should have a error number tho
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
			// returned early
			continue;
		}

        if (msg.connection_fd == -1) {
            // messages with -1 connfd are not valid
        	free(msg.bufferptr);
            continue;
        }

		if(send(msg.connection_fd, msg.bufferptr, msg.packet_len, 0) == -1) { // opts = 0
			// TODO: In all of these cases, the message is getting lost
	    	switch(errno) {
	    	    case EBADF:
	    	        // TODO invalid socket, letting receive task clean up the connection
	    	        break;
	    	    case EINPROGRESS:
	    	        // TODO internal netconn is connecting, closing, or writing in a different place
	    	        break;
	    	    case ENOTCONN:
	    	        // TODO invalid netconn pcb
	    	        break;
	    	    case EIO:
	    	        // TODO NULL netconn, this should be caught in EBADF but in a rare case this might happen
	    	        break;
	    	    case EINVAL:
	    	        // TODO internal overflow or NULL pointer
	    	        break;
	    	    default:
	    	    	// TODO unknown error when sending data, use errno in message
	    	        break;
	    	}
		}
		free(msg.bufferptr);

	}
    // TODO Log exiting writer thread due to a shutdown, could be error or status, should have a error number tho
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
	if(is_server_running()) {
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
	if(is_server_running()) {
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

// compares the connections list to the fd_set and sets the fd_set to the active connections
int update_fd_set(fd_set *rfds) {
    FD_ZERO(rfds);

	sys_mutex_lock(conn_mu);

	for (int i = 0; i < MAX_CONN_NUM; ++i) {
		if (connections[i] != -1) {
			FD_SET(connections[i], rfds);
		}
    }

	sys_mutex_unlock(conn_mu);
    return 0;
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
	if(xSemaphoreTake(runningMutex, 5) == pdPASS) {
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

		// Force reader task out of select()
		sys_mutex_lock(conn_mu);
	    for (int i = 0; i < MAX_CONN_NUM; ++i) {
	    	if(connections[i] != -1) {
	    		close(connections[i]);
	    		connections[i] = -1;
	    	}
	    }
		sys_mutex_unlock(conn_mu);

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
    if(xSemaphoreTake(deviceMutex, portMAX_DELAY) == pdPASS) {
    	int fd = devices[dev];
    	xSemaphoreGive(deviceMutex);
    	return fd;
    }
    return -2;
}

// Scans connections for bad sockets - sockets that have been closed without setting the corresponding fd to -1
void remove_bad_fds() {
	sys_mutex_lock(conn_mu);
	for(int i = 0; i < MAX_CONN_NUM; ++i) {
		if(connections[i] != -1) {
			int type;
			socklen_t len = sizeof(type);
			if(getsockopt(connections[i], SOL_SOCKET, SO_TYPE, &type, &len) == -1) {
			    if(errno == EBADF) {
			        // TODO: bad fd in list, include fd number
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
			        // TODO: unknown socket error during fd list scan, include fd number and errno number
			    }
			}
		}
    }
	sys_mutex_unlock(conn_mu);
}
