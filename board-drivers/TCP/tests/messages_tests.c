#include "../server/inc/messages.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Helper function to print buffer as hex
void print_buffer(const uint8_t *buffer, int size) {
	printf("Buffer [%d bytes]: ", size);
	for (int i = 0; i < size; i++) {
		printf("%02X ", buffer[i]);
	}
	printf("\n");
}

float random_float(float min, float max) {
	return ((float)rand() / RAND_MAX) * max + min;
}

void test_telemetry_message(BoardId board_id) {
	printf("Testing telemetry message serialization/deserialization...\n");

	int num_channels;
	if (board_id == BOARD_FC) {
		num_channels = NUM_FC_CHANNELS;
	} else if (board_id == BOARD_BAY_1 || board_id == BOARD_BAY_2 ||
			   board_id == BOARD_BAY_3) {
		num_channels = NUM_BAY_CHANNELS;
	} else if (board_id == BOARD_FR) {
		num_channels = NUM_FR_CHANNELS;
	} else {
		printf("Unknown board id: %i", board_id);
		return;
	}

	// Create a telemetry message
	Message original_msg;
	original_msg.type = MSG_TELEMETRY;
	original_msg.data.telemetry.board_id = board_id;
	original_msg.data.telemetry.timestamp = 0x123456789ABCDEF0ULL;
	original_msg.data.telemetry.num_channels = num_channels;

	// Set some test data
	for (int i = 0; i < num_channels; i++) {
		// Not totally sure what the expected range of values are, but hopefully
		// this will do
		original_msg.data.telemetry.telemetry_data[i] =
			random_float(-1000, 1000);
	}

	// Serialize
	uint8_t buffer[MAX_MESSAGE_SIZE];
	int serialized_size =
		serialize_message(&original_msg, buffer, sizeof(buffer));

	printf("Serialized size: %d bytes\n", serialized_size);
	print_buffer(buffer, serialized_size);

	// Deserialize
	Message deserialized_msg;
	int consumed_bytes =
		deserialize_message(buffer, serialized_size, &deserialized_msg);

	printf("Deserialized size: %d bytes\n", consumed_bytes);

	// Verify
	assert(consumed_bytes == serialized_size);
	assert(deserialized_msg.type == original_msg.type);
	assert(deserialized_msg.data.telemetry.board_id ==
		   original_msg.data.telemetry.board_id);
	assert(deserialized_msg.data.telemetry.timestamp ==
		   original_msg.data.telemetry.timestamp);
	assert(deserialized_msg.data.telemetry.num_channels ==
		   original_msg.data.telemetry.num_channels);

	for (uint32_t i = 0; i < original_msg.data.telemetry.num_channels; i++) {
		float diff = original_msg.data.telemetry.telemetry_data[i] -
					 deserialized_msg.data.telemetry.telemetry_data[i];
		assert(fabs(diff) < 0.0001f);
	}

	printf("Telemetry message test passed!\n\n");
}

void test_valve_command_message() {
	printf("Testing valve command message serialization/deserialization...\n");

	// Create a valve command message
	Message original_msg;
	original_msg.type = MSG_VALVE_COMMAND;
	original_msg.data.valve_command.valve_id = 0x12;
	original_msg.data.valve_command.valve_state = 0x01;

	// Serialize
	uint8_t buffer[MAX_MESSAGE_SIZE];
	int serialized_size =
		serialize_message(&original_msg, buffer, sizeof(buffer));

	printf("Serialized size: %d bytes\n", serialized_size);
	print_buffer(buffer, serialized_size);

	// Deserialize
	Message deserialized_msg;
	int consumed_bytes =
		deserialize_message(buffer, serialized_size, &deserialized_msg);

	printf("Deserialized size: %d bytes\n", consumed_bytes);

	// Verify
	assert(consumed_bytes == serialized_size);
	assert(deserialized_msg.type == original_msg.type);
	assert(deserialized_msg.data.valve_command.valve_id ==
		   original_msg.data.valve_command.valve_id);
	assert(deserialized_msg.data.valve_command.valve_state ==
		   original_msg.data.valve_command.valve_state);

	printf("Valve command message test passed!\n\n");
}

void test_valve_state_message() {
	printf("Testing valve state message serialization/deserialization...\n");

	// Create a valve state message
	Message original_msg;
	original_msg.type = MSG_VALVE_STATE;
	original_msg.data.valve_state.valve_id = 0x05;
	original_msg.data.valve_state.valve_state = 0x01;
	original_msg.data.valve_state.timestamp = 0xFEDCBA9876543210ULL;

	// Serialize
	uint8_t buffer[MAX_MESSAGE_SIZE];
	int serialized_size =
		serialize_message(&original_msg, buffer, sizeof(buffer));

	printf("Serialized size: %d bytes\n", serialized_size);
	print_buffer(buffer, serialized_size);

	// Deserialize
	Message deserialized_msg;
	int consumed_bytes =
		deserialize_message(buffer, serialized_size, &deserialized_msg);

	printf("Deserialized size: %d bytes\n", consumed_bytes);

	// Verify
	assert(consumed_bytes == serialized_size);
	assert(deserialized_msg.type == original_msg.type);
	assert(deserialized_msg.data.valve_state.valve_id ==
		   original_msg.data.valve_state.valve_id);
	assert(deserialized_msg.data.valve_state.valve_state ==
		   original_msg.data.valve_state.valve_state);
	assert(deserialized_msg.data.valve_state.timestamp ==
		   original_msg.data.valve_state.timestamp);

	printf("Valve state message test passed!\n\n");
}

void test_heartbeat_message() {
	printf("Testing heartbeat message serialization/deserialization...\n");

	// Create a heartbeat message
	Message original_msg;
	original_msg.type = MSG_HEARTBEAT;

	// Serialize
	uint8_t buffer[MAX_MESSAGE_SIZE];
	int serialized_size =
		serialize_message(&original_msg, buffer, sizeof(buffer));

	printf("Serialized size: %d bytes\n", serialized_size);
	print_buffer(buffer, serialized_size);

	// Deserialize
	Message deserialized_msg;
	int consumed_bytes =
		deserialize_message(buffer, serialized_size, &deserialized_msg);

	printf("Deserialized size: %d bytes\n", consumed_bytes);

	// Verify
	assert(consumed_bytes == serialized_size);
	assert(deserialized_msg.type == original_msg.type);

	printf("Heartbeat message test passed!\n\n");
}

void test_partial_data() {
	printf("Testing partial data handling...\n");
	
	// Create a telemetry message
	Message original_msg;
	original_msg.type = MSG_TELEMETRY;
	original_msg.data.telemetry.board_id = BOARD_FC;
	original_msg.data.telemetry.timestamp = 0x123456789ABCDEF0ULL;
	original_msg.data.telemetry.num_channels = NUM_FC_CHANNELS;
	
	for (int i = 0; i < NUM_FC_CHANNELS; i++) {
		original_msg.data.telemetry.telemetry_data[i] = random_float(-1000, 1000);
	}
	
	// Serialize
	uint8_t buffer[MAX_MESSAGE_SIZE];
	int serialized_size = serialize_message(&original_msg, buffer, sizeof(buffer));
	
	// Test with partial data (only header)
	Message partial_msg;
	int result = deserialize_message(buffer, 1, &partial_msg);
	assert(result == 0); // Should return 0 for "not enough data"
	
	// Test with partial data (header + board_id but not timestamp)
	result = deserialize_message(buffer, 2, &partial_msg);
	assert(result == 0);
	
	printf("Partial data handling test passed!\n\n");
}

void test_edge_cases() {
	printf("Testing edge cases...\n");

	// Test buffer too small
	Message msg;
	msg.type = MSG_TELEMETRY;
	msg.data.telemetry.board_id = BOARD_FC;
	msg.data.telemetry.timestamp = 0x123456789ABCDEF0ULL;
	msg.data.telemetry.num_channels = NUM_FC_CHANNELS;

	uint8_t small_buffer[5]; // Definitely too small
	int result = serialize_message(&msg, small_buffer, sizeof(small_buffer));
	assert(result == -1); // Should return error

	printf("-Small buffer test passed!\n");

	// Test invalid message type
	uint8_t invalid_buffer[10] = {0xFF, 0xf0, 0x6c, 0x00, 0x00,
								  0x00, 0x00, 0x00, 0x00, 0x00};
	Message invalid_msg;
	result = deserialize_message(invalid_buffer, sizeof(invalid_buffer),
								 &invalid_msg);
	assert(result == -1); // Should return error

	printf("-Invalid message type test passed!\n");
	
	// Test invalid board id in telemetry message
	uint8_t invalid_board_buffer[20] = {
		MSG_TELEMETRY, 0xFF, /* Invalid board ID */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Timestamp */
		0x00, 0x00, 0x00, 0x00 /* First float value */
	};
	result = deserialize_message(invalid_board_buffer, sizeof(invalid_board_buffer), &invalid_msg);
	assert(result == -1); // Should return error
	
	printf("-Invalid board ID test passed!\n");

	// Test too many channels
	msg.data.telemetry.num_channels = MAX_TELEMETRY_CHANNELS + 1;
	uint8_t buffer[MAX_MESSAGE_SIZE];
	result = serialize_message(&msg, buffer, sizeof(buffer));
	assert(result == -1); // Should return error
	
	printf("-Too many channels test passed!\n");

	printf("Edge cases tests passed!\n\n");
}

void test_buffer_boundary_conditions() {
	printf("Testing buffer boundary conditions...\n");
	
	// Test with buffer size exactly equal to the message size
	Message msg;
	msg.type = MSG_VALVE_COMMAND;
	msg.data.valve_command.valve_id = 0x01;
	msg.data.valve_command.valve_state = 0x01;
	
	uint8_t buffer[MAX_VALVE_COMMAND_MSG_SIZE];
	int serialized_size = serialize_message(&msg, buffer, MAX_VALVE_COMMAND_MSG_SIZE);
	assert(serialized_size == MAX_VALVE_COMMAND_MSG_SIZE);
	
	// Test with buffer size exactly one byte less than needed
	serialized_size = serialize_message(&msg, buffer, MAX_VALVE_COMMAND_MSG_SIZE - 1);
	assert(serialized_size == -1); // Should fail
	
	printf("Buffer boundary conditions test passed!\n\n");
}

int main() {
	printf("Starting message serialization/deserialization tests\n");
	printf("===================================================\n\n");

	test_telemetry_message(BOARD_FC);
	test_telemetry_message(BOARD_BAY_1);
	test_telemetry_message(BOARD_FR);
	test_valve_command_message();
	test_valve_state_message();
	test_heartbeat_message();
	test_partial_data();
	test_edge_cases();
	test_buffer_boundary_conditions();

	printf("All tests passed successfully!\n");
	return 0;
}