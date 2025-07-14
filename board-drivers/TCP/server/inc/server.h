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

#define MAX_MSG_LEN	300
#define TCP_KEEP_ALIVE_IDLE	5
#define TCP_KEEP_ALIVE_INTERVAL	3
#define TCP_KEEP_ALIVE_COUNT 3

typedef enum {
	LimeWire_d = 0U,
	BayBoard1_d,
	BayBoard2_d,
	BayBoard3_d,
	FlightRecorder_d
} Target_Device;

// Raw message structure, used for low-level communication
typedef struct {
	int connection_fd; // file descriptor for the connection
	char buffer[MAX_MSG_LEN];
	int packet_len; // length of the packet
} Raw_message;

// Function to initialize the server. Returns -1 if server is already running
int server_init(void);

// Listen for incoming connections and spawn reader and writer threads for each connection
void server_listener_thread(void *arg);

// A reader for a single connection, parses and executes commands
void server_reader_thread(void *arg);

// A writer for a single connection, sends telemetry data
void server_writer_thread(void *arg);

void msgFreeCallback(void * data);

// Function to read a message from the server
// Returns -1 if server is not running or block = 0 and no message is available
int server_read(Raw_message *msg, TickType_t block);

// Returns -1 if server is not running
int server_send(Raw_message* msg, TickType_t block);

int update_fd_set(fd_set *rfds);

// Is server running. Returns 1 if server is running, 0 if server is stopped, -1 on error
int is_server_running();

// Shutdown server. Returns 0 on success, -1 if the server is not running
int shutdown_server();

// Create server semaphores and mutexes. Should only be called once and be called before anything else in this file
int server_create(ip4_addr_t limewire, ip4_addr_t bb1, ip4_addr_t bb2, ip4_addr_t bb3, ip4_addr_t fr);

// Get the fd of a connected device. Returns the fd if the device is connected, -1 if the device is not connected
int get_device_fd(Target_Device dev);

#endif

