/**
 * Header file for communicating with W25N02GV Flash Memory
 * Datasheet: https://www.winbond.com/resource-files/W25N02GV%20revf%20050918%20unsecured.pdf
 *
 * NOTE: The W25N02GV is 2 W25N01GV's in a trenchcoat, so this "library"
 * is just a wrapper around the W25N01GV functions.
 *
 * The references to "fc_flash" in this library are because this particular chip
 * is used on the TSM flight computer, as opposed to regular "flash" being
 * used on other TSM/MASA boards.
 *
 * Nathaniel Kalantar (nkalan@umich.edu)
 * Michigan Aeronautical Science Association
 * Created February 21, 2021
 * Last edited February 21, 2021
 */

#ifndef W25N02GV_H	// Begin header include protection
#define W25N02GV_H

#include "main.h"

#ifdef HAL_SPI_MODULE_ENABLED	// Begin SPI include protection

#include "../inc/W25N01GV.h"

typedef struct {
	W25N01GV_Flash flash0;
	W25N01GV_Flash flash1;

	SPI_HandleTypeDef *SPI_bus;   // SPI struct, specified by user
	GPIO_TypeDef *cs_base;        // Chip select GPIO base, specified by user
	uint16_t cs_pin;              // Chip select GPIO pin, specified by user

	uint8_t current_write_die;    // Track the die that has the last written address
	uint8_t current_read_die;     // Track the die that has the next page to read

	HAL_StatusTypeDef last_HAL_status;
} W25N02GV_Flash;

/**
 * Initializes the flash memory chip with SPI and pin information,
 * sets parameters to an initial state, enables the onboard
 * ECC and buffer read mode, and finds the address of the first
 * location in memory available to be written to.
 *
 * Selects whichever die contains the first available address.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param SPI_bus_in <SPI_HandleTypeDef*> Struct used for SPI communication
 * @param cs_base    <GPIO_TypeDef*>      GPIO pin array the chip select pin is on
 * @param cs_pin     <uint16_t>           GPIO pin connected to flash chip select
 */
void fc_init_flash(W25N02GV_Flash *fc_flash, SPI_HandleTypeDef *SPI_bus_in,
		GPIO_TypeDef *cs_base_in, uint16_t cs_pin_in);

/**
 * Check that the device's JEDEC ID matches the one listed in the datasheet.
 * Use this function to check if the flash and the SPI bus is functioning.
 *
 * Reads and checks the manufacturer ID and the device ID.
 * datasheet pg 27
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @retval 1 if it read the ID back correctly, 0 if it didn't
 */
uint8_t fc_ping_flash(W25N02GV_Flash *fc_flash);

/**
 * Resets the flash chip to it's power-on state. If the device is busy when
 * this function is called, it will ignore the command and return 0 to avoid
 * corrupting data.
 *
 * Causes a typical delay of 5 microseconds and a max delay of 500 microseconds
 *
 * Note: doesn't track which die was selected previously. Will finish executing
 * with die 1 selected. Call fc_init_flash() again after this function to update.
 *
 * datasheet pg 26
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @retval 1 if the reset command executed, 0 if the device was busy and didn't reset.
 */
uint8_t fc_reset_flash(W25N02GV_Flash *fc_flash);

/**
 * Erase the entire flash memory chip. Erasing means setting every byte to 0xFF.
 * This function also resets the address counters in the two W25N01GV_Flash structs
 * contained in the W25N02GV_Flash struct.
 *
 * WARNING: This function will erase all data, and causes a substantial delay
 * on the order of 2-10 seconds. Only use it if you're absolutely sure.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @retval The number of memory blocks that failed to erase
 */
uint16_t fc_erase_flash(W25N02GV_Flash *fc_flash);

/**
 * Writes data from an array to the W25N01GV flash memory chip.
 * It automatically tracks the address of data it writes; no address
 * management is required by the user. If it runs out of space, it
 * will stop writing data and do nothing.
 *
 * Note: It can only write data to memory locations that were previously
 * erased, so make sure to call erase_flash() once before you start writing.
 *
 * Stores data in the struct's buffer and only transmits periodically,
 * to prevent the ECC from getting corrupted.
 *
 * After you're done with your program and will no longer write to flash,
 * call finish_flash_write() to store the remaining contents of the
 * write buffer to flash.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param data       <uint8_t*>           Array of data to write to flash
 * @param num_bytes  <uint32_t>           Number of bytes to write to flash
 * @retval The number of memory blocks that failed to write
 */
uint16_t fc_write_to_flash(W25N02GV_Flash *fc_flash, uint8_t *data, uint32_t num_bytes);

/**
 * Writes whatever is contained in the write buffer to flash.
 * You MUST call this function at the end of your program, before the
 * W25N10GV_Flash struct goes out of scope, or else you will lose
 * up to the last 512 bytes of data.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
uint16_t fc_finish_flash_write(W25N02GV_Flash *fc_flash);

/**
 * To be used before calling read_next_2KB_from_flash().
 *
 * Sets the page counter in the W25N01GV_Flash struct to the first page
 * of die 0 for reading. Use this first, then call read_next_2KB_from_flash().
 * See README for sample code.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
void fc_reset_flash_read_pointer(W25N02GV_Flash *fc_flash);

/**
 * Reads a 2KB page into the supplied buffer, then increments a counter so it
 * will output the next page the next time this function is called.
 *
 * To read out the entire memory array, call reset_read_pointer(), then call
 * this function up to W25N01GV_NUM_PAGES times. See README for sample code.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param buffer     <uint8_t*>           Buffer to hold 2048 bytes of data
 */
void fc_read_next_2KB_from_flash(W25N02GV_Flash *fc_flash, uint8_t *buffer);

/**
 * Returns the index of the current page. This function exists to abstract
 * away the 2 internal flash chips. If die0 is not full, it returns the current
 * page of die0. If die0 is full, it returns the number of pages in die0 plus
 * the current page of die1.
 *
 * Use the output of this function as the upper limit (inclusive) for loop counters when reading from flash.
 */
uint32_t fc_flash_current_page(W25N02GV_Flash *fc_flash);

/**
 * Returns the number of bytes remaining in the flash memory array that are
 * available to write to.
 *
 * Adds the amount remaining in both flash chips.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @retval Number of free bytes remaining in the flash chip to write to
 */
uint32_t fc_get_bytes_remaining(W25N02GV_Flash *fc_flash);

#endif	// end SPI include protection
#endif	// end header include protection
