/*
 * NEO-M92-00B.c
 *
 *  Created on: Dec 1, 2023
 *      Author: Evan Eidt
 */

#include "../inc/NEO-M92-00B.h"

int init_gps(gps_handler* hgps) {

    hgps->rx_buffer_1_pos = 0;
    hgps->rx_buffer_2_pos = 0;
    hgps->active_rx_buffer = 1;

    uint8_t tx_buffer[BUFFER_SIZE];
    uint8_t rx_buffer[BUFFER_SIZE];

    // Set CFG-NAVSPG-DYNMODEL-AIR4 (<4g) config item to RAM
        // Preamble + Class/ID of command 
    memcpy(tx_buffer, &(uint32_t){0xB562068A}, 4); // Header (2B) + Class [1B] + ID [1B] for VALSET 
        // Length
    tx_buffer[4] = 9; // Length of payload = 4 + (Key-Value pair lengths)
        // Start of payload
    memcpy(tx_buffer + 5, &(uint32_t){0x00010000}, 4); // Version [1B] + RAM layer [1B] + Reserved [2B] for VALSET;
        // Key for config item
    memcpy(tx_buffer + 9, &(uint32_t){UBLOXCFG_CFG_NAVSPG_DYNMODEL_ID}, 4); // copy key to buffer at index 9
        // Value for config item
    tx_buffer[13] = UBLOXCFG_CFG_NAVSPG_DYNMODEL_AIR4; // copy value to index 13
        // Calculate checksum starting from Class field according to 8-bit Fletcher algorithm
    uint8_t CK_A, CK_B = 0;
    for (int i = 2; i < 14; ++i) {
        CK_A += tx_buffer[i]; 
        CK_B += CK_A;
    }
        // Add checksum to tx_buffer
    tx_buffer[14] = CK_A;
    tx_buffer[15] = CK_B;

    HAL_UART_Transmit(hgps->huart, tx_buffer, 16, HAL_MAX_DELAY); // send VALSET command to GPS chip

    // Check for ACK
    HAL_UART_Receive(hgps->huart, rx_buffer, 8, HAL_MAX_DELAY); // receive GPS chip response to VALSET command

    const uint8_t VALSET_ACK[8] = {0xB5, 0x62, 0x05, 0x01, 0x06, 0x8A, 0x96, 0xAD}; // 0x96 0xAD result from calculating checksum
    if (memcmp(rx_buffer, VALSET_ACK, 8)) { // memcmp returns 0 if ACK received for VALSET as expected, returning a truthy value is a failed ACK check
        // Did not receive ACK
        return -1;
    }

    HAL_UART_Receive_IT(hgps->huart, hgps->uart_rx_byte, 1); // Setup UART for interrupt-based rx

    return 0;
}

void irq_gps_callback(gps_handler* hgps) { // TODO make better

    BaseType_t xHigherPriorityTaskWoken = pdFALSE; 

    if (hgps->active_rx_buffer == 1) { // rx_buffer_1 is active
		if(hgps->uart_rx_byte != '$') { // Look for start of next GPS packet ('$' delimits)
			hgps->rx_buffer_1[hgps->rx_buffer_1_pos++] = hgps->uart_rx_byte; // No '$' found, keep adding to rx_buffer_1
		}
		else { // '$' found, switch buffers and release semaphore to start parsing
			hgps->rx_buffer_1[hgps->rx_buffer_1_pos] = '\0'; // Add string delimiter
			
            hgps->active_rx_buffer = 2; // Switch to rx_buffer_2 while rx_buffer_1 gets processed

			hgps->rx_buffer_2[0] = '$'; // Start adding to rx_buffer_2
			hgps->rx_buffer_2_pos = 1;

			xSemaphoreGiveFromISR(hgps->semaphore, &xHigherPriorityTaskWoken); // release semaphore to kick the parsing RTOS task
		}
	}
	else { // rx_buffer_2 is active
		if(hgps->uart_rx_byte != '$') { // Look for start of next GGA packet ('$' delimits)
			hgps->rx_buffer_2[hgps->rx_buffer_2_pos++] = hgps->uart_rx_byte; // No '$' found, keep adding to rx_buffer_2
		}
		else { // '$' found, switch buffers and release semaphore to start parsing
			hgps->rx_buffer_2[hgps->rx_buffer_2_pos] = '\0';
			
            hgps->active_rx_buffer = 1; // Switch to rx_buffer_1 while rx_buffer_2 gets processed

			hgps->rx_buffer_1[0] = '$'; // Start adding to rx_buffer_2
			hgps->rx_buffer_1_pos = 1;

			xSemaphoreGiveFromISR(hgps->semaphore, &xHigherPriorityTaskWoken); // release semaphore to kick the parsing RTOS task
		}
	}
    
	HAL_UART_Receive_IT(hgps->huart, hgps->uart_rx_byte, 1); // Setup UART for interrupt-based rx

    /* Yield if xHigherPriorityTaskWoken is pdTrue.  The 
    actual macro used here is port specific. Initiates a context-switch */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

void parse_gps_sentence(const char* sentence, gps_data* gps) {
	struct minmea_sentence_gga parsed;
	
	if (minmea_parse_gga(&parsed, sentence)) { // Check for valid GGA format sentence
		if (parsed.latitude.scale != 0 && parsed.longitude.scale != 0) { // check for valid scaling to avoid divide-by-zero error

            // Format time in seconds since UTC epoch
			gps->time = (parsed.time.hours * 3600) + (parsed.time.minutes * 60) + parsed.time.seconds + (((double)parsed.time.microseconds) / 1000.0);

            // Convert to Lat/Long in degrees, floating point
			gps->latitude = minmea_tocoord(&parsed.latitude); 
			gps->longitude = minmea_tocoord(&parsed.longitude);

			gps->fix_quality = parsed.fix_quality;
			gps->sats_tracked = parsed.satellites_tracked;

            // Horizontal Dillution of Precision
			float hdop_temp = (float) parsed.hdop.value;
			gps->hdop = hdop_temp / parsed.hdop.scale;

            // Convert to Altitude in meters MSL
			float alt_temp = (float) parsed.altitude.value;
			gps->altitude = alt_temp / parsed.altitude.scale;

			GPS_Log = 1;
		}
	}
}
