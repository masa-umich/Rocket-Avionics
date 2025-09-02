/**
 * Header file for MASA W25N04KV Flash Memory firmware library
 * Datasheet: https://www.winbond.com/resource-files/W25N04KVxxIRU_Datasheet_RevH.pdf
 *
 * The W25N04KV is essentially 4 W25N01GV's in a larger package, so this library
 * is a wrapper around the W25N01GV functions with die selection capability.
 *
 * Author: [Your Name]
 * Michigan Aeronautical Science Association
 * Created March 10, 2025
 * Last edited March 10, 2025
 */

#ifndef W25N04KV_H
#define W25N04KV_H

#include "W25N01GV.h"

/**
 * Structure to store W25N04KV flash memory information
 */
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
} W25N04KV_Flash;

/**
 * Initialize W25N04KV flash memory
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @param SPI_bus_in SPI bus handle
 * @param cs_base_in GPIO port for chip select
 * @param cs_pin_in GPIO pin for chip select
 */
void fc_init_flash(W25N04KV_Flash *fc_flash, SPI_HandleTypeDef *SPI_bus_in,
                  GPIO_TypeDef *cs_base_in, uint16_t cs_pin_in);

/**
 * Ping flash to check if it's alive and responding correctly
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return 1 if flash responds with correct ID, 0 otherwise
 */
uint8_t fc_ping_flash(W25N04KV_Flash *fc_flash);

/**
 * Reset all dies in the flash memory
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return Success status (0 if successful)
 */
uint8_t fc_reset_flash(W25N04KV_Flash *fc_flash);

/**
 * Erase all data in flash memory
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return Number of erase failures
 */
uint16_t fc_erase_flash(W25N04KV_Flash *fc_flash);

/**
 * Write data to flash memory
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @param data Data to write
 * @param num_bytes Number of bytes to write
 * @return Number of write failures
 */
uint16_t fc_write_to_flash(W25N04KV_Flash *fc_flash, uint8_t *data, uint32_t num_bytes);

/**
 * Finish any pending write operations
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return Status of write operation
 */
uint16_t fc_finish_flash_write(W25N04KV_Flash *fc_flash);

/**
 * Reset the read pointer to the beginning of flash
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 */
void fc_reset_flash_read_pointer(W25N04KV_Flash *fc_flash);

/**
 * Read the next 2KB block from flash
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @param buffer Buffer to store read data
 */
void fc_read_next_2KB_from_flash(W25N04KV_Flash *fc_flash, uint8_t *buffer);

/**
 * Get the current page being accessed
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return Current page number (global across all dies)
 */
uint32_t fc_flash_current_page(W25N04KV_Flash *fc_flash);

/**
 * Get total bytes remaining in flash
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return Number of bytes remaining
 */
uint32_t fc_get_bytes_remaining(W25N04KV_Flash *fc_flash);

/**
 * Select a specific die for operations
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @param die_id Die ID (0-3) to select
 */
void select_die(W25N04KV_Flash *fc_flash, uint8_t die_id);

#endif /* W25N04KV_H */
