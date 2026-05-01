/*
 * Current and voltage sensing math, PT math, and other utility functions
 *
 * utils.h
 *
 *  Created on: July 4th, 2025
 *      Author: felixfb
 */
#ifndef PTUTILS_H
#define PTUTILS_H

#include "main.h"
#include "logging.h"
#include "client.h"

void reset_board();

int send_msg_to_device(Message *msg, TickType_t wait);

int send_raw_msg_to_device(RawMessage *msg, TickType_t wait);

#endif
