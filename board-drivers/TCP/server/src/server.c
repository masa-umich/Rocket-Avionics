/*
 * server.c
 *
 *  Created on: Feb 16, 2025
 *      Author: jackmh
 */

#include "server.h"
#include "semphr.h"

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
    rxMsgBuffer = list_create(MAX_MSG_LEN + 2*sizeof(int), msgFreeCallback);
	txMsgBuffer = list_create(MAX_MSG_LEN + 2*sizeof(int), msgFreeCallback);

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
	if(xSemaphoreTake(runningMutex, 1) == pdPASS) {
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
    if (listen_sockfd == -1) {
        Error_Handler(); // TODO: add error handling
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

    if (bind(listen_sockfd, (struct sockaddr *) &server_socket, sizeof(server_socket)) != 0) {
        printf("Error binding socket\n");
        vTaskDelete(NULL);
        Error_Handler(); // TODO: add error handling
    }

    // Listen for incoming connections (blocks until a connection is made)
    if (listen(listen_sockfd, MAX_CONN_NUM) < 0) {
        printf("Error listening on socket\n");
        close(listen_sockfd);
        vTaskDelete(NULL);
        Error_Handler(); // TODO: add error handling
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
        	continue;
        }

        int idle = TCP_KEEP_ALIVE_IDLE;
        int intvl = TCP_KEEP_ALIVE_INTERVAL;
        int optval = 1;
        int probecnt = TCP_KEEP_ALIVE_COUNT;
        setsockopt(connection_fd, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval));
        setsockopt(connection_fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
        setsockopt(connection_fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(connection_fd, IPPROTO_TCP, TCP_KEEPCNT, &probecnt, sizeof(probecnt));

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
    timeout.tv_sec = 0; // 1 second timeout
    timeout.tv_usec = 100000;

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
        if (select(MAX_CONN_NUM+1, &rx_fd_set, NULL, NULL, &timeout) != -1) {
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
                	Raw_message msg = {0};

                    // read the message
				    int packet_len = recv(connection_fd, msg.buffer, MAX_MSG_LEN, 0);

				    if(packet_len <= 0) {
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

				    msg.connection_fd = connection_fd;
				    msg.packet_len = packet_len;

				    // add the message to the RX Message Buffer to be parsed
                    list_push(rxMsgBuffer, (void*)(&msg), portMAX_DELAY);
                }
            }

            // unlock the connections mutex
            sys_mutex_unlock(conn_mu);
        } else {
        	// Link down error, loop to shutdown
        	//Error_Handler(); // TODO: error handling
        }
    }
    xSemaphoreGive(shutdownDone);
	vTaskDelete(NULL);
    // TODO: add error handling and/or freeing memory
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
            continue;
        }

		if (send(msg.connection_fd, msg.buffer, msg.packet_len, 0) == -1) { // opts = 0
			// Do nothing since this most likely is due to a closed connection.
			// Lost telemetry and valve state messages are ok since they will be sent as soon as the connection is reestablished
			// Lost valve commands shouldn't be held to send later since that could be dangerous if a board is reconnected and instantly a valve opens/closes
            //Error_Handler(); // TODO: add error handling
		}

	}

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
			return -1;
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
			return -1;
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
        // TODO: add error handling
    }

	sys_mutex_unlock(conn_mu);
    return 0;
}

int is_server_running() {
	if(xSemaphoreTake(runningMutex, 1) == pdPASS) {
		uint8_t stat = running;
		xSemaphoreGive(runningMutex);
		return stat;
	}
	else {
		return -1;
	}
}

void drain_lists() {
	uint8_t space;
	do {
		Raw_message msg = {0};
		space = !list_pop(txMsgBuffer, (void*)(&msg), 0);
	} while(space);

	do {
		Raw_message msg = {0};
		space = !list_pop(rxMsgBuffer, (void*)(&msg), 0);
	} while(space);
}

int shutdown_server() {
	if(xSemaphoreTake(runningMutex, 1) == pdPASS) {
		if(!running) {
			xSemaphoreGive(runningMutex);
			return -1;
		}
		else {
			running = 0;
			xSemaphoreGive(runningMutex);
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

	    if(xSemaphoreTake(deviceMutex, 10) == pdPASS) {
	    	memset(devices, -1, 5 * sizeof(int));
	    	xSemaphoreGive(deviceMutex);
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
    return -1;
}
