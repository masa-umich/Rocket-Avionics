/**
 * Header file for communicating with an array of W25N02GV flash memory chips.
 * 
 * NOTE: Name of file doesn't correspond to an actual chip; The 'M' is for 'multiple'.
 * 
 * NOTE: Like the W25N02GV file, this file is essentially a wrapper around the W25N01GV functions
 *       except this time there can be as many flash chips as we want! Yay for modularity!
 * 	
 * NOTE: Actually, turns out we only have 4. But you can change that if you want, I guess. You do you.
 * NOTE: Methods to read data off flash chips are not included in this file, as it wouldn't make sense to read from all chips at once.
 * 		 Instead, use the W25N02GV functions to read from individual chips. In addition, the helper methods to get the 
 * 		 current page as well as the memory remaining are not included in this file, as they can just be called on individual chips.
 * 
 * Teja Koduru (tkoduru@umich.edu)
 * Michigan Aeronautical Science Association
 * Created February 19, 2024
 * Last edited February 24, 2021
 */

#ifndef W25N0MGV_H	// Begin header include protection
#define W25N0MGV_H

#include "W25N01GV.h"
#include "W25N02GV.h" // Include the W25N02GV file to control the individual flash chips
#include "main.h"

#ifdef HAL_SPI_MODULE_ENABLED	// Begin SPI include protection (TODO: Check if this is necessary)

// This struct represents multiple flash chips, each with their own SPI bus, chip select pin, and address counters.
// We only have 4 flash chips, so we only need space for 4 here. Keep in mind, if you want to add more flash chips,
// you'll need to add more space in the flashes array. 
// FIXME: This is kinda a hacky way to do this; In the future, we should probably allow for a variable number of flash chips.

#define NUM_FLASHES (uint16_t)4 // Number of flash chips in the array

typedef struct {
	W25N02GV_Flash flashes[NUM_FLASHES];
} W25N0MGV_Flash;


/**
 * Initializes all flash memory chips with SPI and pin information,
 * sets parameters to an initial state, enables the onboard
 * ECC and buffer read mode, and finds the address of the first
 * location in memory available to be written to.
 *
 *
 * @param fc_flashes      <W25N02GV_Flash*>   Struct used to store all flash chips
 * @param SPI_busses_in <SPI_HandleTypeDef*> Array of SPI buses the flash chips are on
 * @param cs_bases_in    <GPIO_TypeDef*>      Array of GPIO bases connected to flash chips
 * @param cs_pins_in     <uint16_t*>           Array of pin numbers connected to flash chips
 */

void fc_init_flashes(W25N02GV_Flash *fc_flashes, SPI_HandleTypeDef *SPI_busses_in,
		GPIO_TypeDef *cs_bases_in, uint16_t *cs_pins_in);

/**
 * Check all flash chips contained within the W25N0MGV_Flash struct to make sure 
 * that both the flash chip and the SPI bus are functioning.
 * 
 * More information on the JEDEC ID can be found in the datasheet on pg 27.
 *
 * @param fc_flashes     <W25N0MGV_Flash*>    Struct used to store all flash chips
 * @retval 1 if all flash chips are functioning, 0 if any flash chip is not functioning
 */

uint8_t fc_ping_flashes(W25N0MGV_Flash *storage);

/**
 * Resets all flash memory chips contained within the W25N0MGV_Flash struct.
 * See datasheet pg 26 for more information, as well as the W25N02GV.h file.
 *
 * @param fc_flashes    Struct used to store all flash chips
 * @retval 1 if all flash chips were reset, 0 if any flash chip was not reset
 */
uint8_t fc_reset_flashes(W25N0MGV_Flash *storage);

/**
 * Erase all flash memory chips contained within the W25N0MGV_Flash struct.
 *
 * WARNING: This function will erase all data, and causes a substantial delay
 * on the order of 2-10 seconds. Only use it if you're absolutely sure.
 *
 * @param fc_flashes    Struct used to store all flash chips
 * @retval The number of memory blocks across all flash chips that failed to erase (Max: 2 * NUM_FLASHES)
 */
uint16_t fc_erase_flashes(W25N0MGV_Flash *storage);

/**
 * 
 * Writes data to all flash memory chips contained within the W25N0MGV_Flash struct.
 * Please consult the W25N02GV.h file for more information on how this function works for each 
 * individual flash chip.
 * 
 * NOTE: Please make sure to call erase_flash() once before you start writing.
 * In addition, make sure to call finish_flash_write() at the end of your program.
 *
 * @param fc_flashes    Struct used to store all flash chips
 * @param data       <uint8_t*>           Array of data to write to each flash
 * @param num_bytes  <uint32_t>           Number of bytes to write to each flash
 * @retval The number of memory blocks across all chips that failed to write
 */
uint16_t fc_write_to_flashes(W25N0MGV_Flash *storage, uint8_t *data, uint32_t num_bytes);

/**
 * Writes all remaining data in the write buffer to all flash chips, and then 
 * resets the write buffer. You MUST call this function at the end of your program,
 * before the flash structs go out of scope, or else you will lose up to the last 512 bytes of data.
 *
 * @param fc_flashes    Struct used to store all flash chips
 * @retval The number of flash chips that failed to finish writing
 */
uint16_t fc_finish_flash_writes(W25N0MGV_Flash *storage);


#endif 		// End SPI include protection	
#endif    // End header include protection
