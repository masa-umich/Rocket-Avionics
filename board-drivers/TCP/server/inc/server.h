/*
 * server.h
 *
 *  Created on: Feb 16, 2025
 *      Author: jackmh
 */

#pragma once
#ifndef INC_TCPSERVER_H_
#define INC_TCPSERVER_H_

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/sockets.h"
#include "lwip.h"
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "tsqueue.h"

// Raw message structure, used for low-level communication
typedef struct {
	int connection_fd; // file descriptor for the connection
	char *buffer; // pointer to the message buffer
	int packet_len; // length of the packet
} Raw_message;

// Function to initialize the server
int server_init(void);

// Listen for incoming connections and spawn reader and writer threads for each connection
void server_listener_thread(void *arg);

// A reader for a single connection, parses and executes commands
void server_reader_thread(void *arg);

// A writer for a single connection, sends telemetry data
void server_writer_thread(void *arg);

void msgFreeCallback(void * data);

// Function to read a message from the server
int server_read(Raw_message *msg);

int server_send(Raw_message* msg);

int update_fd_set(fd_set *rfds);

#endif

