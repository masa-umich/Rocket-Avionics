/**
 * Implementation for MASA W25N04KV Flash Memory firmware library
 * Datasheet: https://www.winbond.com/resource-files/W25N04KVxxIRU_Datasheet_RevH.pdf
 *
 * The W25N04KV is essentially 4 W25N01GV's in a larger package, so this library
 * is a wrapper around the W25N01GV functions with die selection capability.
 *
 * Author: Jack Hammerberg, Nathaniel Kalantar (and Claude 3.7 lol)
 * Michigan Aeronautical Science Association
 * Created March 10, 2025
 * Last edited March 10, 2025
 */

#include "W25N04KV.h"

// Die selection command and die IDs
#define W25N04KV_DIE_SELECT (uint8_t) 0xC2
#define W25N04KV_DIE_0 (uint8_t) 0x00
#define W25N04KV_DIE_1 (uint8_t) 0x01
#define W25N04KV_DIE_2 (uint8_t) 0x02
#define W25N04KV_DIE_3 (uint8_t) 0x03

// JEDEC ID values for verification
#define W25N04KV_READ_JEDEC_ID (uint8_t) 0x9F
#define W25N04KV_MANUFACTURER_ID (uint8_t) 0xEF
#define W25N04KV_DEVICE_ID (uint16_t) 0xAA23

// Chip select states
#define W25N04KV_CS_ACTIVE (uint8_t) GPIO_PIN_RESET
#define W25N04KV_CS_INACTIVE (uint8_t) GPIO_PIN_SET

// SPI timeout value
#define W25N04KV_SPI_TIMEOUT (uint8_t) 0x64

/**
 * Selects which die to communicate with
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @param die_id Die ID (0-3) to select
 */
void select_die(W25N04KV_Flash *fc_flash, uint8_t die_id) {
    uint8_t tx[2] = { W25N04KV_DIE_SELECT, die_id };
    
    __disable_irq();
    HAL_GPIO_WritePin(fc_flash->cs_base, fc_flash->cs_pin, W25N04KV_CS_ACTIVE); // Select chip
    // Transmit/receive, and store the status code
    fc_flash->last_HAL_status = HAL_SPI_Transmit(fc_flash->SPI_bus, tx, 2, W25N04KV_SPI_TIMEOUT);
    HAL_GPIO_WritePin(fc_flash->cs_base, fc_flash->cs_pin, W25N04KV_CS_INACTIVE); // Release chip
    __enable_irq();
}

/**
 * Initialize W25N04KV flash memory
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @param SPI_bus_in SPI bus handle
 * @param cs_base_in GPIO port for chip select
 * @param cs_pin_in GPIO pin for chip select
 */
void fc_init_flash(W25N04KV_Flash *fc_flash, SPI_HandleTypeDef *SPI_bus_in,
                  GPIO_TypeDef *cs_base_in, uint16_t cs_pin_in) {
    fc_flash->SPI_bus = SPI_bus_in;
    fc_flash->cs_base = cs_base_in;
    fc_flash->cs_pin = cs_pin_in;
    
    // Initialize all four dies
    select_die(fc_flash, 0);
    init_flash(&fc_flash->flash0, SPI_bus_in, cs_base_in, cs_pin_in);
    
    select_die(fc_flash, 1);
    init_flash(&fc_flash->flash1, SPI_bus_in, cs_base_in, cs_pin_in);
    
    select_die(fc_flash, 2);
    init_flash(&fc_flash->flash2, SPI_bus_in, cs_base_in, cs_pin_in);
    
    select_die(fc_flash, 3);
    init_flash(&fc_flash->flash3, SPI_bus_in, cs_base_in, cs_pin_in);
    
    fc_flash->current_read_die = 0;
    
    // Find the first die that isn't full for writing
    if (get_bytes_remaining(&fc_flash->flash0) != 0) {
        select_die(fc_flash, 0);
        fc_flash->current_write_die = 0;
    }
    else if (get_bytes_remaining(&fc_flash->flash1) != 0) {
        select_die(fc_flash, 1);
        fc_flash->current_write_die = 1;
    }
    else if (get_bytes_remaining(&fc_flash->flash2) != 0) {
        select_die(fc_flash, 2);
        fc_flash->current_write_die = 2;
    }
    else {
        // Already selected die 3 above
        fc_flash->current_write_die = 3;
    }
}

/**
 * Ping flash to check if it's alive and responding correctly
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return 1 if flash responds with correct ID, 0 otherwise
 */
uint8_t fc_ping_flash(W25N04KV_Flash *fc_flash) {
    uint8_t tx[2] = { W25N04KV_READ_JEDEC_ID, 0 }; // Second byte unused
    uint8_t rx[3];
    
    __disable_irq();
    HAL_GPIO_WritePin(fc_flash->cs_base, fc_flash->cs_pin, W25N04KV_CS_ACTIVE); // Select chip
    // Transmit/receive, and store the status code
    fc_flash->last_HAL_status = HAL_SPI_Transmit(fc_flash->SPI_bus, tx, 2, W25N04KV_SPI_TIMEOUT);
    fc_flash->last_HAL_status = HAL_SPI_Receive(fc_flash->SPI_bus, rx, 3, W25N04KV_SPI_TIMEOUT);
    HAL_GPIO_WritePin(fc_flash->cs_base, fc_flash->cs_pin, W25N04KV_CS_INACTIVE); // Release chip
    __enable_irq();
    
    uint8_t manufacturer_ID = rx[0];
    uint16_t device_ID = (rx[1] << 8) + rx[2];
    
    if (manufacturer_ID == W25N04KV_MANUFACTURER_ID && device_ID == W25N04KV_DEVICE_ID)
        return 1;
    else
        return 0;
}

/**
 * Reset all dies in the flash memory
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return Success status (0 if successful)
 */
uint8_t fc_reset_flash(W25N04KV_Flash *fc_flash) {
    // Reset all four flash dies to their power on state
    uint8_t success_status = 0;
    
    select_die(fc_flash, 0);
    success_status |= reset_flash(&fc_flash->flash0);
    
    select_die(fc_flash, 1);
    success_status |= reset_flash(&fc_flash->flash1);
    
    select_die(fc_flash, 2);
    success_status |= reset_flash(&fc_flash->flash2);
    
    select_die(fc_flash, 3);
    success_status |= reset_flash(&fc_flash->flash3);
    
    return success_status;
}

/**
 * Erase all data in flash memory
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return Number of erase failures
 */
uint16_t fc_erase_flash(W25N04KV_Flash *fc_flash) {
    uint16_t erase_failures = 0;
    
    // Erase all four dies
    select_die(fc_flash, 3);
    erase_failures += erase_flash(&fc_flash->flash3);
    
    select_die(fc_flash, 2);
    erase_failures += erase_flash(&fc_flash->flash2);
    
    select_die(fc_flash, 1);
    erase_failures += erase_flash(&fc_flash->flash1);
    
    select_die(fc_flash, 0);
    erase_failures += erase_flash(&fc_flash->flash0);
    
    // Reset write pointers
    fc_flash->current_write_die = 0;
    fc_flash->current_read_die = 0;
    
    return erase_failures;
}

/**
 * Write data to flash memory
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @param data Data to write
 * @param num_bytes Number of bytes to write
 * @return Number of write failures
 */
uint16_t fc_write_to_flash(W25N04KV_Flash *fc_flash, uint8_t *data, uint32_t num_bytes) {
    uint16_t write_failures = 0;
    
    // Write to current die, moving to next die if needed
    while (num_bytes > 0) {
        uint32_t current_die_bytes_remaining = 0;
        
        // Get remaining bytes in current die
        switch (fc_flash->current_write_die) {
            case 0:
                current_die_bytes_remaining = get_bytes_remaining(&fc_flash->flash0);
                break;
            case 1:
                current_die_bytes_remaining = get_bytes_remaining(&fc_flash->flash1);
                break;
            case 2:
                current_die_bytes_remaining = get_bytes_remaining(&fc_flash->flash2);
                break;
            case 3:
                current_die_bytes_remaining = get_bytes_remaining(&fc_flash->flash3);
                break;
        }
        
        // If current die is full, move to next die
        if (current_die_bytes_remaining == 0) {
            fc_flash->current_write_die++;
            if (fc_flash->current_write_die > 3) {
                // All dies are full
                break;
            }
            select_die(fc_flash, fc_flash->current_write_die);
            continue;
        }
        
        // Determine how many bytes to write to current die
        uint32_t bytes_to_write = (num_bytes <= current_die_bytes_remaining) ? 
                                  num_bytes : current_die_bytes_remaining;
        
        // Write to current die
        switch (fc_flash->current_write_die) {
            case 0:
                write_failures += write_to_flash(&fc_flash->flash0, data, bytes_to_write);
                break;
            case 1:
                write_failures += write_to_flash(&fc_flash->flash1, data, bytes_to_write);
                break;
            case 2:
                write_failures += write_to_flash(&fc_flash->flash2, data, bytes_to_write);
                break;
            case 3:
                write_failures += write_to_flash(&fc_flash->flash3, data, bytes_to_write);
                break;
        }
        
        // Update pointers
        data += bytes_to_write;
        num_bytes -= bytes_to_write;
        
        // If current die is now full, prepare to move to next die
        if (bytes_to_write == current_die_bytes_remaining && num_bytes > 0) {
            fc_flash->current_write_die++;
            if (fc_flash->current_write_die > 3) {
                // All dies are full
                break;
            }
            select_die(fc_flash, fc_flash->current_write_die);
        }
    }
    
    return write_failures;
}

/**
 * Finish any pending write operations
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return Status of write operation
 */
uint16_t fc_finish_flash_write(W25N04KV_Flash *fc_flash) {
    // Finish write on current die
    uint16_t status = 0;
    
    switch (fc_flash->current_write_die) {
        case 0:
            status = finish_flash_write(&fc_flash->flash0);
            // If die0 is now full, move to die1
            if (get_bytes_remaining(&fc_flash->flash0) == 0) {
                select_die(fc_flash, 1);
                fc_flash->current_write_die = 1;
            }
            break;
        case 1:
            status = finish_flash_write(&fc_flash->flash1);
            // If die1 is now full, move to die2
            if (get_bytes_remaining(&fc_flash->flash1) == 0) {
                select_die(fc_flash, 2);
                fc_flash->current_write_die = 2;
            }
            break;
        case 2:
            status = finish_flash_write(&fc_flash->flash2);
            // If die2 is now full, move to die3
            if (get_bytes_remaining(&fc_flash->flash2) == 0) {
                select_die(fc_flash, 3);
                fc_flash->current_write_die = 3;
            }
            break;
        case 3:
            status = finish_flash_write(&fc_flash->flash3);
            break;
    }
    
    return status;
}

/**
 * Reset the read pointer to the beginning of flash
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 */
void fc_reset_flash_read_pointer(W25N04KV_Flash *fc_flash) {
    // Select die 0 and reset its read pointer
    select_die(fc_flash, 0);
    reset_flash_read_pointer(&fc_flash->flash0);
    fc_flash->current_read_die = 0;
}

/**
 * Read the next 2KB block from flash
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @param buffer Buffer to store read data
 */
void fc_read_next_2KB_from_flash(W25N04KV_Flash *fc_flash, uint8_t *buffer) {
    // Read from current die, moving to next die if needed
    switch (fc_flash->current_read_die) {
        case 0:
            read_next_2KB_from_flash(&fc_flash->flash0, buffer);
            // If reached end of die0, move to die1
            if (fc_flash->flash0.next_page_to_read >= W25N01GV_NUM_PAGES) {
                select_die(fc_flash, 1);
                reset_flash_read_pointer(&fc_flash->flash1);
                fc_flash->current_read_die = 1;
            }
            break;
        case 1:
            read_next_2KB_from_flash(&fc_flash->flash1, buffer);
            // If reached end of die1, move to die2
            if (fc_flash->flash1.next_page_to_read >= W25N01GV_NUM_PAGES) {
                select_die(fc_flash, 2);
                reset_flash_read_pointer(&fc_flash->flash2);
                fc_flash->current_read_die = 2;
            }
            break;
        case 2:
            read_next_2KB_from_flash(&fc_flash->flash2, buffer);
            // If reached end of die2, move to die3
            if (fc_flash->flash2.next_page_to_read >= W25N01GV_NUM_PAGES) {
                select_die(fc_flash, 3);
                reset_flash_read_pointer(&fc_flash->flash3);
                fc_flash->current_read_die = 3;
            }
            break;
        case 3:
            read_next_2KB_from_flash(&fc_flash->flash3, buffer);
            break;
    }
}

/**
 * Get the current page being accessed
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return Current page number (global across all dies)
 */
uint32_t fc_flash_current_page(W25N04KV_Flash *fc_flash) {
    if (fc_flash->current_write_die == 0) {
        return fc_flash->flash0.current_page;
    }
    else if (fc_flash->current_write_die == 1) {
        return W25N01GV_NUM_PAGES + fc_flash->flash1.current_page;
    }
    else if (fc_flash->current_write_die == 2) {
        return (2 * W25N01GV_NUM_PAGES) + fc_flash->flash2.current_page;
    }
    else { // die 3
        return (3 * W25N01GV_NUM_PAGES) + fc_flash->flash3.current_page;
    }
}

/**
 * Get total bytes remaining in flash
 * 
 * @param fc_flash Struct used to store flash pins and addresses
 * @return Number of bytes remaining
 */
uint32_t fc_get_bytes_remaining(W25N04KV_Flash *fc_flash) {
    // Sum the bytes remaining in all four dies
    return get_bytes_remaining(&fc_flash->flash0) + 
           get_bytes_remaining(&fc_flash->flash1) +
           get_bytes_remaining(&fc_flash->flash2) +
           get_bytes_remaining(&fc_flash->flash3);
}
