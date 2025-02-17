/*
 * server.c
 *
 *  Created on: Feb 16, 2025
 *      Author: jackmh
 */

#include "server.h"

#define SERVER_PORT 5000
#define MAX_CONN_NUM 8

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
        (writerTaskHandle == NULL)) 
    {
        return 1;
    } else {
        return 0;
    }
}

// Listen for incoming connections and add them to the connections list
void server_listener_thread(void *arg) {

    int listen_sockfd = socket(AF_INET, SOCK_STREAM, 6); // Create new socket for listening. 6 is the protocol # for tcp
    if (listen_sockfd == -1) {
        return -1;
    }

    // Bind socket to port
    struct sockaddr_in server_socket;
    server_socket.sin_family = AF_INET;
    server_socket.sin_port = htons(SERVER_PORT);
    server_socket.sin_addr.s_addr = (in_addr_t) ipaddr.addr;

    if (bind(listen_sockfd, (struct sockaddr *)&server_socket, sizeof(server_socket)) < 0) {
        printf("Error binding socket\n");
        vTaskDelete(NULL);
        return 1;
    }

    // Listen for incoming connections (blocks until a connection is made)
    if (listen(listen_sockfd, MAX_CONN_NUM) < 0) {
        printf("Error listening on socket\n");
        close(listen_sockfd);
        vTaskDelete(NULL);
        return 1;
    }

    for (;;) {
        // we have a connection!
        struct sockaddr_in connection_socket;

        int connection_fd = accept(listen_sockfd, (struct sockaddr *)&connection_socket, &sizeof(connection_socket));
    
        // handle the connection
        // TODO: the rest of the code lol
    }

}

// Continually reads from the RX Message Buffer and processes new messages
void server_reader_thread(void *arg) {

}

// Continually writes to the TX Message Buffer
void server_writer_thread(void *arg) {

}

// Adds a message to the TX Message Buffer to be sent to a client
int sever_send(/* args that make sense once I write more code */) {
    return 1;
}