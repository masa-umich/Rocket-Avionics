
/* EEPROM header file
 * created: 03/20/2024
 * author: Luke Orlebeke
 */
#ifndef M23256E_H //header file protection
#define M23256E_H




#define eeprom_address C0C1C20101 //C0, C1, C2 depend on the manufacturing number of our specific
								  // eeprom

#define num_pages 500			  // calculated value, not explicitly mentioned in the user
								  // manual

#define bytes_per_page 64		  // number of bytes in a page

#define I2C_handle
int // i2c handle definition goes here

void eeprom_read(uint16_t page_num, uint8_t offset, uint8_t num_bytes, uint16_t *data);

//will read configuration data from the eeprom, takes the offset within the page, the number of
//bytes you want to read, the page number you want to read from, and a pointer to the data buffer
//as parameters.


void eeprom_write(uint16_t page_num, uint8_t offset, uint8_t num_bytes, uint16_t *data);

//identical to the read function but it writes and also needs a HAL_delay of 5 ms bc
//write cycle takes 5 ms


void flash_write(uint32_t sector, uint32_t *data, uint16_t num_words);

//writes to flash in stm


uint8_t available_bytes(uint8_t offset, uint8_t num_bytes);
