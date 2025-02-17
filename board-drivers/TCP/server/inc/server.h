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

// Function to initialize the server
int server_init(void);

// Listen for incoming connections and spawn reader and writer threads for each connection
void server_listen_thread(void *arg);

// A reader for a single connection, parses and executes commands
void server_reader_thread(void *arg);

// A writer for a single connection, sends telemetry data
void server_writer_thread(void *arg);

#endif

