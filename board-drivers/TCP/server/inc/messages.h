/*
 * messages.h
 *
 *  Created on: Jan 26, 2024
 *      Author: tiger
 */

#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>

// Message types (and header bytes)
typedef enum { MSG_TELEMETRY = 0x01, MSG_VALVE_COMMAND = 0x02, MSG_HEARTBEAT = 0x03 } MessageType;

// Board identifiers
typedef enum {
	BOARD_FC = 0x00,
	BOARD_BAY_1 = 0x01,
	BOARD_BAY_2 = 0x02,
	BOARD_BAY_3 = 0x03
} BoardId;

// Telemetry message
#define MAX_TELEMETRY_CHANNELS 52
#define NUM_FC_CHANNELS 47
#define NUM_BAY_CHANNELS 52
typedef struct {
	BoardId board_id;
	uint64_t timestamp;
	float telemetry_data[MAX_TELEMETRY_CHANNELS];
	uint32_t num_channels;
} TelemetryMessage;

// Valve command message
typedef struct {
	uint32_t command_bitmask;
	uint32_t state_bitmask;
} ValveCommandMessage;

// Heartbeat message (no data)
typedef struct {
	// No additional data
} HeartbeatMessage;

#define MAX_TELEMETRY_MSG_SIZE (1 + 1 + 8 + (4 * MAX_TELEMETRY_CHANNELS))
#define MAX_VALVE_COMMAND_MSG_SIZE (1 + 4 + 4)
#define MAX_HEARTBEAT_MSG_SIZE 1
#define MAX_MESSAGE_SIZE MAX_TELEMETRY_MSG_SIZE
typedef struct {
	MessageType type;
	union {
		TelemetryMessage telemetry;
		ValveCommandMessage valve_command;
		HeartbeatMessage heartbeat;
	} data;
} Message;

int serialize_message(const Message *message, uint8_t *buffer, uint32_t buffer_size);
int deserialize_message(const uint8_t *buffer, uint32_t buffer_size, Message *msg);

#endif
