/**
 * Implementation MASA W25N01GV Flash Memory firmware library
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
 *
 */

#include "../inc/W25N02GV.h"

// Switch between the two W25N01GV chips
#define W25N02GV_DIE_SELECT                       (uint8_t) 0xC2
#define W25N02GV_DIE_0                            (uint8_t) 0x00
#define W25N02GV_DIE_1                            (uint8_t) 0x01

// Used to ping flash to check if it's alive
#define W25N02GV_READ_JEDEC_ID                    (uint8_t) 0x9F
#define	W25N02GV_MANUFACTURER_ID                  (uint8_t) 0xEF
#define W25N02GV_DEVICE_ID                       (uint16_t) 0xAB21

// Chip is active low
#define W25N02GV_CS_ACTIVE                        (uint8_t)  GPIO_PIN_RESET
#define W25N02GV_CS_INACTIVE                      (uint8_t)  GPIO_PIN_SET

// Arbitrary timeout value
#define W25N02GV_SPI_TIMEOUT                      (uint8_t)  0xFF

void select_die(W25N02GV_Flash *fc_flash, uint8_t die_id) {
	uint8_t tx[2] = { W25N02GV_DIE_SELECT, die_id };

	__disable_irq();
	HAL_GPIO_WritePin(fc_flash->cs_base, fc_flash->cs_pin, W25N02GV_CS_ACTIVE);  // Select chip
	// Transmit/receive, and store the status code
	fc_flash->last_HAL_status = HAL_SPI_Transmit(fc_flash->SPI_bus, tx, 2, W25N02GV_SPI_TIMEOUT);
	HAL_GPIO_WritePin(fc_flash->cs_base, fc_flash->cs_pin, W25N02GV_CS_INACTIVE);  // Release chip
	__enable_irq();
}

void fc_init_flash(W25N02GV_Flash *fc_flash, SPI_HandleTypeDef *SPI_bus_in,
		GPIO_TypeDef *cs_base_in, uint16_t cs_pin_in) {
	select_die(fc_flash, 0);
	init_flash(&fc_flash->flash0, SPI_bus_in, cs_base_in, cs_pin_in);
	select_die(fc_flash, 1);
	init_flash(&fc_flash->flash1, SPI_bus_in, cs_base_in, cs_pin_in);

	fc_flash->current_read_die = 0;

	// write pointers were already found by init_flash
	// if die0 isn't full, select it.
	// if it is, switch to die1.
	if (get_bytes_remaining(&fc_flash->flash0) != 0) {
		select_die(fc_flash, 0);
		fc_flash->current_write_die = 0;
	}
	else {
		//select_die(fc_flash, 1);  // Already selected above
		fc_flash->current_write_die = 1;
	}
}

uint8_t fc_ping_flash(W25N02GV_Flash *fc_flash) {
	uint8_t tx[2] = { W25N02GV_READ_JEDEC_ID, 0 };  // Second byte unused
	uint8_t rx[3];

	__disable_irq();
	HAL_GPIO_WritePin(fc_flash->cs_base, fc_flash->cs_pin, W25N02GV_CS_ACTIVE);  // Select chip
	// Transmit/receive, and store the status code
	fc_flash->last_HAL_status = HAL_SPI_Transmit(fc_flash->SPI_bus, tx, 2, W25N02GV_SPI_TIMEOUT);
	fc_flash->last_HAL_status = HAL_SPI_Receive(fc_flash->SPI_bus, rx, 3, W25N02GV_SPI_TIMEOUT);
	HAL_GPIO_WritePin(fc_flash->cs_base, fc_flash->cs_pin, W25N02GV_CS_INACTIVE);  // Release chip
	__enable_irq();

	uint8_t manufacturer_ID = rx[0];
	uint16_t device_ID = (rx[1] << 8) + rx[0];

	if (manufacturer_ID == W25N02GV_MANUFACTURER_ID && device_ID == W25N02GV_DEVICE_ID)
		return 1;
	else
		return 0;
}

uint8_t fc_reset_flash(W25N02GV_Flash *fc_flash) {
	// Reset both flash chips to their power on state.
	// Note: this function doesn't track which die was previously selected

	uint8_t success_status = 0;

	select_die(fc_flash, 0);
	success_status |= reset_flash(&fc_flash->flash0);
	select_die(fc_flash, 1);
	success_status |= reset_flash(&fc_flash->flash1);

	return success_status;
}

uint16_t fc_erase_flash(W25N02GV_Flash *fc_flash) {
	uint16_t erase_failures = 0;

	// select die 1 and call erase on it, then
	// select die 0 and call erase on it.
	select_die(fc_flash, 1);
	erase_failures += erase_flash(&fc_flash->flash1);
	select_die(fc_flash, 0);
	erase_failures += erase_flash(&fc_flash->flash0);

	// erase_flash() automatically resets the write pointers in flash structs
	fc_flash->current_write_die = 0;
	fc_flash->current_read_die = 0;

	return erase_failures;
}

uint16_t fc_write_to_flash(W25N02GV_Flash *fc_flash, uint8_t *data, uint32_t num_bytes) {
	uint16_t write_failures = 0;

	// If die0 is selected
	if (fc_flash->current_write_die == 0 && num_bytes > 0) {
		uint32_t die0_num_bytes = get_bytes_remaining(&fc_flash->flash0);

		// If you run out of room on die0, write what you can and switch to die1
		if (die0_num_bytes <= num_bytes) {
			write_failures += write_to_flash(&fc_flash->flash0, data, die0_num_bytes);
			data += die0_num_bytes;
			num_bytes -= die0_num_bytes;

			select_die(fc_flash, 1);
			fc_flash->current_write_die = 1;
			// The last if statement in this function will write the rest to die1

		}
		// If there's enough room in die0
		else {
			write_failures += write_to_flash(&fc_flash->flash0, data, num_bytes);
		}
	}

	// If die1 is selected, either before this function call or because
	// it was selected by the previous if statement, write to it.
	if (fc_flash->current_write_die == 1 && num_bytes > 0) {
		write_failures += write_to_flash(&fc_flash->flash1, data, num_bytes);
	}

	return write_failures;
}

uint16_t fc_finish_flash_write(W25N02GV_Flash *fc_flash) {
	// do finish_write_flash on current die
	if (fc_flash->current_write_die == 0) {
		finish_flash_write(&fc_flash->flash0);

		// if this causes die0 to fill up, switch to die1.
		if (get_bytes_remaining(&fc_flash->flash0) == 0) {
			select_die(fc_flash, 1);
			fc_flash->current_write_die = 1;
		}
	}
	else {  // else if (fc_flash->current_write_die == 1)
		finish_flash_write(&fc_flash->flash1);
	}

	return fc_flash->flash0.last_write_failure_status | fc_flash->flash1.last_write_failure_status;
}

void fc_reset_flash_read_pointer(W25N02GV_Flash *fc_flash) {
	// select die 0 and reset its read pointer
	select_die(fc_flash, 0);
	reset_flash_read_pointer(&fc_flash->flash0);
	fc_flash->current_read_die = 0;
}

void fc_read_next_2KB_from_flash(W25N02GV_Flash *fc_flash, uint8_t *buffer) {
	// call read_next_2kb_from_flash on current die
	if (fc_flash->current_read_die == 0) {
		read_next_2KB_from_flash(&fc_flash->flash0, buffer);

		// if it gets to the last page of die 0,
		// switch to die 1 and call reset_flash_read_pointer on it.
		if (fc_flash->flash0.next_page_to_read >= W25N01GV_NUM_PAGES) {
			select_die(fc_flash, 1);
			reset_flash_read_pointer(&fc_flash->flash1);
			fc_flash->current_read_die = 1;
		}
	}
	else {
		read_next_2KB_from_flash(&fc_flash->flash1, buffer);
	}
}

uint32_t fc_flash_current_page(W25N02GV_Flash *fc_flash) {
	if (get_bytes_remaining(&fc_flash->flash0) == 0) {
		return W25N01GV_NUM_PAGES + fc_flash->flash1.current_page;
	}
	else {
		return fc_flash->flash0.current_page;
	}
}

uint32_t fc_get_bytes_remaining(W25N02GV_Flash *fc_flash) {
	// return sum of get_bytes_remaining() for both flash structs.
	return get_bytes_remaining(&fc_flash->flash0) + get_bytes_remaining(&fc_flash->flash1);
}
