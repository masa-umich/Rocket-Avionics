/*
 * LPS22HBTR.h
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#ifndef LPS22HBTR_H	// Begin header include protection
#define LPS22HBTR_H

#include "W25N01GV.h"

typedef struct {
    // The four virtual W25N01GV dies
    W25N01GV_Flash flash0;
    W25N01GV_Flash flash1;
    W25N01GV_Flash flash2;
    W25N01GV_Flash flash3;
    
    // SPI bus and chip select information
    SPI_HandleTypeDef *SPI_bus;
    GPIO_TypeDef *cs_base;
    uint16_t cs_pin;
    
    // Current die information
    uint8_t current_write_die;
    uint8_t current_read_die;
    
    // Status information
    HAL_StatusTypeDef last_HAL_status;
} flash;

typedef struct {
	// Data buffer to store data before writing
	uint8_t write_buffer[W25N01GV_SECTOR_SIZE];

	uint32_t next_page_to_read;   // Tracking pages while reading

	SPI_HandleTypeDef *SPI_bus;   // SPI struct, specified by user
	GPIO_TypeDef *cs_base;        // Chip select GPIO base, specified by user
	uint16_t cs_pin;              // Chip select GPIO pin, specified by user

	uint16_t write_buffer_size;   // Tracking the bytes stored in the buffer while writing

	uint16_t current_page;        // Tracking pages while writing
	uint16_t next_free_column;    // Tracking columns while writing

	// The firmware checks various status codes, all of
	// which can be accessed at any time.
	// (For these four, 0 is good, anything else is bad)
	// TODO: write in the header comments which of these each functions updates
	HAL_StatusTypeDef last_HAL_status;
	W25N01GV_ECC_Status last_read_ECC_status;
	uint8_t last_write_failure_status;
	uint8_t last_erase_failure_status;

} W25N01GV_Flash;

#endif // end header include protection