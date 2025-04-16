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

// Function prototypes
void server_listener_thread(void *arg);
void server_reader_thread(void *arg);
void server_writer_thread(void *arg);
void msgFreeCallback(void * data);
int sever_send(/* args that make sense once I write more code */);

// Function to initialize the server
// Spins off a listener, reader, and writer task/thread 
// as well as initializes the message buffers and connection list
int server_init(void) {
    osThreadId_t listenerTaskHandle, readerTaskHandle, writerTaskHandle;
    // task configuration
    const osThreadAttr_t listenerTask_attributes = {
      .name = "listenerTask",
      .stack_size = 512 * 4,
      .priority = (osPriority_t) osPriorityNormal,
    };
    const osThreadAttr_t readerTask_attributes = {
      .name = "readerTask",
      .stack_size = 512 * 4,
      .priority = (osPriority_t) osPriorityNormal,
    };
    const osThreadAttr_t writerTask_attributes = {
      .name = "writerTask",
      .stack_size = 512 * 4,
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
    // Create new socket for listening. 6 is the protocol # for tcp
    int listen_sockfd = socket(AF_INET, SOCK_STREAM, 6);
    if (listen_sockfd == -1) {
    	vTaskDelete(NULL);
        //return -1;
    }

    // Bind socket to port
    struct sockaddr_in server_socket;
    server_socket.sin_family = AF_INET; // IPv4
    server_socket.sin_port = htons(SERVER_PORT); // grab server port
    server_socket.sin_addr.s_addr = (in_addr_t) ipaddr.addr; // get IP from extern

    if (bind(listen_sockfd, (struct sockaddr *)&server_socket, sizeof(server_socket)) < 0) {
        printf("Error binding socket\n");
        vTaskDelete(NULL);
        //return 1;
    }

    // Listen for incoming connections (blocks until a connection is made)
    if (listen(listen_sockfd, MAX_CONN_NUM) < 0) {
        printf("Error listening on socket\n");
        close(listen_sockfd);
        vTaskDelete(NULL);
        //return 1;
    }

    for (;;) {
        // we have a connection!
        struct sockaddr_in connection_socket;

        socklen_t socket_len = sizeof(connection_socket);
        int connection_fd = accept(listen_sockfd, (struct sockaddr *)&connection_socket, &socket_len);
    
        // add the new connection to the active connections list
        sys_mutex_lock(conn_mu);
	    for (int i = 0; i < MAX_CONN_NUM; ++i) {
	    	if (connections[i] == -1) { // -1 is an empty space in the list
	    		connections[i] = connection_fd;
	    		sys_mutex_unlock(conn_mu);
	    	}
	    }
	    sys_mutex_unlock(conn_mu);
    }
}

// Continually reads from the RX Message Buffer and processes new messages
// Also removes them from the buffer after processing
void server_reader_thread(void *arg) {

    for (;;) {
        // capture incoming messages

        // parse incoming messages

        // execute commands
    }
}

// Continually checks for new messages in the TX Message Buffer and sends them to the client
// Also removes them from the buffer after sending
void server_writer_thread(void *arg) {

    for (;;) {
        // grab new messages on the queue

        // send messages to the client

    	// remove from queue
    }
}

// Callback function to free message data
// TBH I don't get why this is needed but it's for the TS-queue
void msgFreeCallback(void * data) {
	free(data);
}

// Helper function to add a message to the TX Message Buffer
int sever_send(/* args that make sense once I write more code */) {
    return 1;
}
