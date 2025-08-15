/*
 * messages.h
 *
 *  Created on: Jan 26, 2024
 *      Author: tiger
 */

#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>
#include <lmp_channels.h>

// Message types (values are corresponding header bytes)
typedef enum {
	MSG_TELEMETRY = 0x00,
	MSG_VALVE_COMMAND = 0x01,
	MSG_VALVE_STATE = 0x02,
	MSG_HEARTBEAT = 0x03,
	MSG_DEVICE_COMMAND = 0x04
} MessageType;

// Board identifiers for TelemetryMessages and DeviceCommandMessages (values are corresponding bytes)
typedef enum {
	BOARD_FC = 0x00,	// Flight Computer
	BOARD_BAY_1 = 0x01, // Bay Board 1
	BOARD_BAY_2 = 0x02, // Bay Board 2
	BOARD_BAY_3 = 0x03, // Bay Board 3
	BOARD_FR = 0x04,	// Flight Recorder
} BoardId;

typedef enum {
	DEVICE_CMD_RESET = 0x00,
	DEVICE_CMD_CLEAR_FLASH = 0x01,
	DEVICE_CMD_QUERY_FLASH = 0x02
} DeviceCMDId;

#define MAX_TELEMETRY_CHANNELS MAX_TELEM_CHANNELS
#define NUM_FC_CHANNELS FC_TELEMETRY_CHANNELS
#define NUM_BAY_CHANNELS BB1_TELEMETRY_CHANNELS
#define NUM_FR_CHANNELS FR_TELEMETRY_CHANNELS
typedef struct {
	BoardId board_id;
	uint64_t timestamp;
	float telemetry_data[MAX_TELEMETRY_CHANNELS];
	uint32_t num_channels;
} TelemetryMessage;

typedef struct {
	uint8_t valve_id;
	uint8_t valve_state;
} ValveCommandMessage;

typedef struct {
	uint8_t valve_id;
	uint8_t valve_state;
	uint64_t timestamp;
} ValveStateMessage;

typedef struct {
	// No data
} HeartbeatMessage;

typedef struct {
	BoardId board_id;
	DeviceCMDId cmd_id;
} DeviceCommandMessage;

// Message Length + Message Type + Board ID + Timestamp + Channels (float)
#define MAX_TELEMETRY_MSG_SIZE (1 + 1 + 1 + 8 + (4 * MAX_TELEMETRY_CHANNELS))
// Message Length + Message Type + Valve ID + Valve State
#define MAX_VALVE_COMMAND_MSG_SIZE (1 + 1 + 1 + 1)
// Message Length + Message Type + Valve ID + Valve State + Timestamp
#define MAX_VALVE_STATE_MSG_SIZE (1 + 1 + 1 + 1 + 8)
// Message Length + Message Type
#define MAX_HEARTBEAT_MSG_SIZE 2
// Message Length + Message Type + Board ID + Command ID
#define MAX_DEVICE_COMMAND_MSG_SIZE (1 + 1 + 1 + 1)
// Telemetry is the largest message in all cases
#define MAX_MESSAGE_SIZE MAX_TELEMETRY_MSG_SIZE

typedef struct {
	MessageType type;
	union {
		TelemetryMessage telemetry;
		ValveCommandMessage valve_command;
		ValveStateMessage valve_state;
		HeartbeatMessage heartbeat;
		DeviceCommandMessage device_command;
	} data;
} Message;

int serialize_message(const Message *message, uint8_t *buffer,
					  uint32_t buffer_size);
int deserialize_message(const uint8_t *buffer, uint32_t buffer_size,
						Message *msg);
#endif
