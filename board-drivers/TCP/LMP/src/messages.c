/*
 * messages.c
 *
 *  Created on: Jan 26, 2024
 *      Author: tiger
 */

#include "../inc/messages.h"

#include <stdint.h>
#include <string.h>

// Utility functions for endianness conversion (I think either STM32 or lwip
// might have these written, but I'm just going to leave these here for now)
// Change to __REV?

// Convert an 8-byte big-endian array to a uint64_t.
static uint64_t parse_uint64_be(const uint8_t *buf) {
	uint64_t value = 0;
	for (int i = 0; i < 8; i++) {
		value = (value << 8) | buf[i];
	}
	return value;
}

// Convert a 4-byte big-endian array to a uint32_t.
static uint32_t parse_uint32_be(const uint8_t *buf) {
	return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
		   ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}

// Write a uint64_t into buf in big-endian order.
static void pack_uint64_be(uint8_t *buf, uint64_t value) {
	for (int i = 7; i >= 0; i--) {
		buf[i] = value & 0xFF;
		value >>= 8;
	}
}

// Write a uint32_t into buf in big-endian order.
static void pack_uint32_be(uint8_t *buf, uint32_t value) {
	buf[0] = (value >> 24) & 0xFF;
	buf[1] = (value >> 16) & 0xFF;
	buf[2] = (value >> 8) & 0xFF;
	buf[3] = value & 0xFF;
}

typedef union {
	float f;
	uint32_t i;
} FloatConverter;

// Serialize
// Function returns -1 on failure, and number of bytes serialized on success

int serialize_telemetry(const TelemetryMessage *message, uint8_t *buffer,
						uint32_t buffer_size) {
	if (message->num_channels > MAX_TELEMETRY_CHANNELS) {
		return -1; // Too many telemetry channels
	}

	int num_bytes = 1 + 1 + 1 + 8 + 4 * message->num_channels;
	if (buffer_size < num_bytes) {
		return -1; // Buffer size too small
	}

	// First byte already set by serialize_message()
	buffer[0] = (uint8_t) (num_bytes - 1);
	buffer[2] = (uint8_t)message->board_id;
	pack_uint64_be(&buffer[3], message->timestamp);

	for (uint32_t i = 0; i < message->num_channels; i++) {
		FloatConverter converter;
		converter.f = message->telemetry_data[i];
		pack_uint32_be(&buffer[11 + 4 * i], converter.i);
	}

	return num_bytes;
}

int serialize_valve_command(const ValveCommandMessage *message, uint8_t *buffer,
							uint32_t buffer_size) {
	int num_bytes = MAX_VALVE_COMMAND_MSG_SIZE;
	if (buffer_size < num_bytes) {
		return -1;
	}

	buffer[0] = (uint8_t) (num_bytes - 1);
	buffer[2] = message->valve_id;
	buffer[3] = message->valve_state;

	return num_bytes;
}

int serialize_valve_state(const ValveStateMessage *message, uint8_t *buffer,
						  uint32_t buffer_size) {
	int num_bytes = MAX_VALVE_STATE_MSG_SIZE;
	if (buffer_size < num_bytes) {
		return -1;
	}

	buffer[0] = (uint8_t) (num_bytes - 1);
	buffer[2] = message->valve_id;
	buffer[3] = message->valve_state;
	pack_uint64_be(&buffer[4], message->timestamp);

	return num_bytes;
}

int serialize_device_command(const DeviceCommandMessage *message, uint8_t *buffer,
							uint32_t buffer_size) {
	int num_bytes = MAX_DEVICE_COMMAND_MSG_SIZE;
	if (buffer_size < num_bytes) {
		return -1;
	}

	buffer[0] = (uint8_t) (num_bytes - 1);
	buffer[2] = message->board_id;
	buffer[3] = message->cmd_id;

	return num_bytes;
}

int serialize_message(const Message *message, uint8_t *buffer,
					  uint32_t buffer_size) {
	if (buffer_size < 2)
		return -1;

	buffer[1] = (uint8_t)message->type;
	switch (message->type) {
		case MSG_TELEMETRY:
			return serialize_telemetry(&message->data.telemetry, buffer,
									   buffer_size);
		case MSG_VALVE_COMMAND:
			return serialize_valve_command(&message->data.valve_command, buffer,
										   buffer_size);
		case MSG_VALVE_STATE:
			return serialize_valve_state(&message->data.valve_state, buffer,
										 buffer_size);
		case MSG_HEARTBEAT:
			return 1; // No data to serialize

		case MSG_DEVICE_COMMAND:
			return serialize_device_command(&message->data.device_command, buffer,
					 	 	 	 	 	 buffer_size);
		default:
			return -1; // Unknown message type
	}
}

// Deserialize (return: 0 => not enough data, -1 => error, >0 => num bytes
// consumed)
int deserialize_telemetry(const uint8_t *buffer, uint32_t buffer_size,
						  TelemetryMessage *message) {
	// Check if enough data for board_id (1), and timestamp (8)
	int num_bytes = 1 + 1 + 1 + 8;
	if (buffer_size < num_bytes) {
		return 0;
	}

	BoardId board_id = (BoardId)buffer[2];
	message->board_id = board_id;
	message->timestamp = parse_uint64_be(&buffer[3]);

	uint32_t num_channels;
	if (board_id == BOARD_FC) {
		num_channels = NUM_FC_CHANNELS;
	} else if (board_id == BOARD_BAY_1 || board_id == BOARD_BAY_2 ||
			   board_id == BOARD_BAY_3) {
		num_channels = NUM_BAY_CHANNELS;
	} else if (board_id == BOARD_FR) {
		num_channels = NUM_FR_CHANNELS;
	} else {
		// Unknown board id
		return -1;
	}
	message->num_channels = num_channels;

	// Check if enough data for telemetry
	num_bytes = 1 + 1 + 1 + 8 + 4 * num_channels;
	if (buffer_size < num_bytes) {
		return 0;
	}

	for (uint32_t i = 0; i < message->num_channels; i++) {
		FloatConverter converter;
		converter.i = parse_uint32_be(&buffer[11 + 4 * i]);
		message->telemetry_data[i] = converter.f;
	}

	return num_bytes;
}

int deserialize_valve_command(const uint8_t *buffer, uint32_t buffer_size,
							  ValveCommandMessage *msg) {
	int num_bytes = MAX_VALVE_COMMAND_MSG_SIZE;
	if (buffer_size < num_bytes) {
		return 0;
	}

	msg->valve_id = buffer[2];
	msg->valve_state = buffer[3];

	return num_bytes;
}

int deserialize_valve_state(const uint8_t *buffer, uint32_t buffer_size,
							ValveStateMessage *msg) {
	int num_bytes = MAX_VALVE_STATE_MSG_SIZE;
	if (buffer_size < num_bytes) {
		return 0;
	}

	msg->valve_id = buffer[2];
	msg->valve_state = buffer[3];
	msg->timestamp = parse_uint64_be(&buffer[4]);

	return num_bytes;
}

int deserialize_device_command(const uint8_t *buffer, uint32_t buffer_size,
							DeviceCommandMessage *msg) {
	int num_bytes = MAX_DEVICE_COMMAND_MSG_SIZE;
	if (buffer_size < num_bytes) {
		return 0;
	}

	msg->board_id = buffer[2];
	msg->cmd_id = buffer[3];

	return num_bytes;
}

int deserialize_message(const uint8_t *buffer, uint32_t buffer_size,
						Message *msg) {
	if (buffer_size < 2)
		return 0;

	msg->type = (MessageType)buffer[1];
	switch (msg->type) {
		case MSG_TELEMETRY:
			return deserialize_telemetry(buffer, buffer_size,
										 &msg->data.telemetry);
		case MSG_VALVE_COMMAND:
			return deserialize_valve_command(buffer, buffer_size,
											 &msg->data.valve_command);
		case MSG_VALVE_STATE:
			return deserialize_valve_state(buffer, buffer_size,
										   &msg->data.valve_state);
		case MSG_HEARTBEAT:
			return 1; // No data to deserialize

		case MSG_DEVICE_COMMAND:
			return deserialize_device_command(buffer, buffer_size,
					   	   	   	   	   	   &msg->data.device_command);
		default:
			return -1; // Unknown message type
	}
}
