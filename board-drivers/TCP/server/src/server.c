/*
 * server.c
 *
 *  Created on: Feb 16, 2025
 *      Author: jackmh
 */

#include "server.h"

extern ip4_addr_t ipaddr; // get our IP address from somewhere (defined in the IOC)
#define SERVER_PORT 5000
#define MAX_CONN_NUM 8

#define MAX_MSG_LEN 300 // I just took this from the old code, this might need to be larger
List* rxMsgBuffer; // thread-safe queue for incoming messages
List* txMsgBuffer; 

int connections[MAX_CONN_NUM]; // List of active connections
sys_mutex_t* conn_mu; // Mutex for the connections list

// Function to initialize the server
// Spins off a listener, reader, and writer task/thread 
// as well as initializes the message buffers and connection list
int server_init(void) {
    osThreadId_t listenerTaskHandle, readerTaskHandle, writerTaskHandle;
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

    // make thread-safe queues for incoming and outgoing messages
    rxMsgBuffer = list_create(MAX_MSG_LEN + 2*sizeof(int), msgFreeCallback);
	txMsgBuffer = list_create(MAX_MSG_LEN + 2*sizeof(int), msgFreeCallback);

    // make mutex for active connections
    // Note: this doesn't need to be a ts-queue it's just a list of 
    // which connections are active where order does not matter
    memset(connections, -1, MAX_CONN_NUM * sizeof(int)); // fill non-active connections with -1
	conn_mu = malloc(sizeof(sys_mutex_t));

    // check if the mutex got created successfully
    if (sys_mutex_new(conn_mu) != ERR_OK) {
        return 1;
    } else {
        return 0;
    }
}

// Listen for incoming connections and add them to the connections list
void server_listener_thread(void *arg) {
    LWIP_UNUSED_ARG(arg);

    // Create new socket for listening. 6 is the protocol # for tcp
    int listen_sockfd = socket(AF_INET, SOCK_STREAM, 6);
    if (listen_sockfd == -1) {
        Error_Handler(); // TODO: add error handling
    }

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
        // we have a connection!
        struct sockaddr_in connection_socket;
        socklen_t addr_len = sizeof(connection_socket);

        int connection_fd = accept(listen_sockfd, (struct sockaddr* ) &connection_socket, &addr_len);
    
        // add the new connection to the active connections list
        sys_mutex_lock(conn_mu);
	    for (int i = 0; i < MAX_CONN_NUM; ++i) {
	    	if (connections[i] == -1) { // -1 is an empty space in the list
	    		connections[i] = connection_fd;
                break;
	    	}
	    }
	    sys_mutex_unlock(conn_mu);
    }
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
    timeout.tv_sec = 1; // 1 second timeout
    timeout.tv_usec = 0;

    for (;;) {
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
				    char* buf = malloc(MAX_MSG_LEN);

                    // read the message
				    int packet_len = recv(connection_fd, buf, MAX_MSG_LEN, 0);

				    // package it in a struct
				    Raw_message msg = {connection_fd, buf, packet_len};

				    // add the message to the RX Message Buffer to be parsed
                    list_push(rxMsgBuffer, (void*)(&msg));
                }
            }

            // unlock the connections mutex
            sys_mutex_unlock(conn_mu);
        } else {
        	Error_Handler(); // TODO: error handling
        }
    }

    // TODO: add error handling and/or freeing memory
}

// Continually checks for new messages in the TX Message Buffer and sends them to the client
// Also removes them from the buffer after sending
void server_writer_thread(void *arg) {
	LWIP_UNUSED_ARG(arg);

	char buf[MAX_MSG_LEN];

	for (;;) {
		Raw_message msg = {-1, buf, MAX_MSG_LEN};

        // blocks until a message is available in the TX Message Buffer
		list_pop(txMsgBuffer, (void*)(&msg));

        if (msg.connection_fd == -1) {
            // messages with -1 connfd are not valid
            continue;
        }

		if (send(msg.connection_fd, msg.buffer, msg.packet_len, 0) == -1) { // opts = 0
            Error_Handler(); // TODO: add error handling
		}

		free(msg.buffer);
	}

	vTaskDelete(NULL);
}

// Callback function to free message data
// TBH I don't get why this is needed but it's for the TS-queue
void msgFreeCallback(void * data) {
	free(data);
}

int server_read(Raw_message* msg) {
    // Pop a message from the RX Message Buffer
    // This will block until a message is available
    list_pop(rxMsgBuffer, (void*)msg);

    // Return the length of the message
    return msg->packet_len;
}

// Helper function to add a message to the TX Message Buffer
int server_send(Raw_message* msg) {
    // Push a message to the TX Message Buffer
    // This will block until there is space in the buffer
    list_push(txMsgBuffer, (void*)msg);

    return 0; // success
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
