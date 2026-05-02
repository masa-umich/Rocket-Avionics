/*
 * uart-data-struct.h
 *
 *  Created on: Apr 15, 20256
 *      Author: felix
 */

#ifdef __cplusplus
extern "C" {
#endif

#define UART_DATA_LENGTH		(size_t)57
#define UART_HEADER_LENGTH		(size_t)4
#define UART_PACKET_LENGTH 		UART_DATA_LENGTH + UART_HEADER_LENGTH

const uint32_t UART_PACKET_HEADER = 0xE4A28BF4;

typedef struct {
	float gps_lat; // degrees
	float gps_long; // degrees
	uint16_t gps_alt; // meters
	int16_t fc_imu_x; // cm/s/s
	int16_t fc_imu_y; // cm/s/s
	int16_t fc_imu_z; // cm/s/s
	int16_t fc_w_x; // dps
	int16_t fc_w_y; // dps
	int16_t fc_w_z; // dps
	uint16_t fc_bar1; // daPa (P / 10) this is kinda dumb but it preserves the most information in 16 bits
	uint16_t fc_bar2; // daPa
	uint16_t fc_24v; // mV
	int16_t fc_temp; // C
	uint8_t fc_fsm_state; // FSM state
	uint8_t fc_flutus; // bool
	uint16_t chamber_pres; // psi
	uint16_t fuel_pres; // psi
	uint16_t ox_pres; // psi
	uint16_t copv_pres; // psi
	uint8_t recovery_pres; // psi
	float rssi;
	float snr;

	uint64_t ms_epoch; // milliseconds epoch
} UART_Data_t;

static inline int serialize_radio(UART_Data_t * data_h, uint8_t * buf, size_t len) {
	if(len < UART_PACKET_LENGTH) {
		return -1;
	}

	memcpy(buf + 0, &(data_h->gps_lat), 4);
	memcpy(buf + 4, &(data_h->gps_long), 4);
	memcpy(buf + 8, &(data_h->gps_alt), 2);
	memcpy(buf + 10, &(data_h->fc_imu_x), 2);
	memcpy(buf + 12, &(data_h->fc_imu_y), 2);
	memcpy(buf + 14, &(data_h->fc_imu_z), 2);
	memcpy(buf + 16, &(data_h->fc_w_x), 2);
	memcpy(buf + 18, &(data_h->fc_w_y), 2);
	memcpy(buf + 20, &(data_h->fc_w_z), 2);
	memcpy(buf + 22, &(data_h->fc_bar1), 2);
	memcpy(buf + 24, &(data_h->fc_bar2), 2);
	memcpy(buf + 26, &(data_h->fc_24v), 2);
	memcpy(buf + 28, &(data_h->fc_temp), 2);
	memcpy(buf + 30, &(data_h->fc_fsm_state), 1);
	memcpy(buf + 31, &(data_h->fc_flutus), 1);
	memcpy(buf + 32, &(data_h->chamber_pres), 2);
	memcpy(buf + 34, &(data_h->fuel_pres), 2);
	memcpy(buf + 36, &(data_h->ox_pres), 2);
	memcpy(buf + 38, &(data_h->copv_pres), 2);
	memcpy(buf + 40, &(data_h->recovery_pres), 1);
	memcpy(buf + 41, &(data_h->rssi), 4);
	memcpy(buf + 45, &(data_h->snr), 4);

	memcpy(buf + 49, &(data_h->ms_epoch), 8);

	memcpy(buf + 57, &UART_PACKET_HEADER, sizeof(UART_PACKET_HEADER));

	return UART_PACKET_LENGTH;
}

static inline int deserialize_radio(UART_Data_t * data_h, uint8_t * buf, size_t len) {
	if(len < UART_PACKET_LENGTH) {
		return -1;
	}
	if(memcmp(buf + UART_DATA_LENGTH, &UART_PACKET_HEADER, sizeof(UART_PACKET_HEADER)) != 0) {
		return -1;
	}

	memcpy(&(data_h->gps_lat), buf + 0, 4);
	memcpy(&(data_h->gps_long), buf + 4, 4);
	memcpy(&(data_h->gps_alt), buf + 8, 2);
	memcpy(&(data_h->fc_imu_x), buf + 10, 2);
	memcpy(&(data_h->fc_imu_y), buf + 12, 2);
	memcpy(&(data_h->fc_imu_z), buf + 14, 2);
	memcpy(&(data_h->fc_w_x), buf + 16, 2);
	memcpy(&(data_h->fc_w_y), buf + 18, 2);
	memcpy(&(data_h->fc_w_z), buf + 20, 2);
	memcpy(&(data_h->fc_bar1), buf + 22, 2);
	memcpy(&(data_h->fc_bar2), buf + 24, 2);
	memcpy(&(data_h->fc_24v), buf + 26, 2);
	memcpy(&(data_h->fc_temp), buf + 28, 2);
	memcpy(&(data_h->fc_fsm_state), buf + 30, 1);
	memcpy(&(data_h->fc_flutus), buf + 31, 1);
	memcpy(&(data_h->chamber_pres), buf + 32, 2);
	memcpy(&(data_h->fuel_pres), buf + 34, 2);
	memcpy(&(data_h->ox_pres), buf + 36, 2);
	memcpy(&(data_h->copv_pres), buf + 38, 2);
	memcpy(&(data_h->recovery_pres), buf + 40, 1);
	memcpy(&(data_h->rssi), buf + 41, 4);
	memcpy(&(data_h->snr), buf + 45, 4);

	memcpy(&(data_h->ms_epoch), buf + 49, 8);

	return UART_PACKET_LENGTH;
}

#ifdef __cplusplus
}
#endif
