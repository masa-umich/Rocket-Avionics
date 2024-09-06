/**
 * Implementation of the W25N02GV flash chip driver.
 * 
 * NOTE: Name of file doesn't correspond to an actual chip; The 'M' is for 'multiple'.
 * 
 * NOTE: This is a wrapper around the W25N02GV driver, containing multiple W25N02GV chips.
 * 
 * Teja Koduru (tkoduru@umich.edu)
 * Michigan Aeronautical Science Association
 * Created February 19, 2024
 * Last edited February 24, 2021
 */

#include "../inc/W25N0MGV.h"

// TODO: Check include paths to make sure we actually need all of these things

void fc_init_flashes(W25N02GV_Flash *storage, SPI_HandleTypeDef *SPI_busses_in, GPIO_TypeDef *cs_bases_in, uint16_t *cs_pins_in) {
	W25N02GV_Flash* fc_flashes = storage;
	for(int i = 0; i < NUM_FLASHES; i++) {
		fc_init_flash(&fc_flashes[i], &SPI_busses_in[i], &cs_bases_in[i], cs_pins_in[i]);
	}
}

uint8_t fc_ping_flashes(W25N0MGV_Flash *storage){
	uint8_t status = 0;
	W25N02GV_Flash* fc_flashes = storage;
	for(int i = 0; i < NUM_FLASHES; i++) {
		status |= fc_ping_flash(&fc_flashes[i]);
	}
	return status;
}

uint8_t fc_reset_flashes(W25N0MGV_Flash *storage){
	uint8_t status = 0;
	W25N02GV_Flash* fc_flashes = storage;
	for(int i = 0; i < NUM_FLASHES; i++) {
		status |= fc_reset_flash(&fc_flashes[i]);
	}
	return status;
}

uint16_t fc_erase_flashes(W25N0MGV_Flash *storage){
	uint16_t status = 0;
	W25N02GV_Flash* fc_flashes = storage;
	for(int i = 0; i < NUM_FLASHES; i++) {
		status += fc_erase_flash(&fc_flashes[i]);
	}
	return status;
}

uint16_t fc_write_to_flashes(W25N0MGV_Flash *storage, uint8_t *data, uint32_t num_bytes){
	uint16_t status = 0;
	W25N02GV_Flash* fc_flashes = storage;
	for(int i = 0; i < NUM_FLASHES; i++) {
		status += fc_write_to_flash(&fc_flashes[i], data, num_bytes);
	}
	return status;
}

uint16_t fc_finish_flash_writes(W25N0MGV_Flash *storage){
	uint16_t status = 0;
	W25N02GV_Flash* fc_flashes = storage;
	for(int i = 0; i < NUM_FLASHES; i++) {
		status += fc_finish_flash_write(&fc_flashes[i]);
	}
	return status;
}
