/* EEPROM driver file
 * created 03/21/2024
 * author: Luke Orlebeke
 *
 */

#include M23256E.h

uint8_t available_bytes(uint8_t offset, uint8_t num_bytes){
	if ((num_bytes + offset)<bytes_per_page){
		return num_bytes;
		else return bytes_per_page - offset;
	}

}

void eeprom_read(uint16_t page_num, uint8_t offset, uint8_t num_bytes, uint16_t *data){

	uint16_t left_shift = 6;

	uint16_t start_page = page_num;

	uint16_t end_page = start_page + ((offset+num_bytes)/bytes_per_page);

	uint16_t num_pages = (end_page - start_page) + 1;

	int pos = 0;

	while (start_page<num_pages){

		uint16_t mem_address = page_num<<left_shift;

		uint8_t bytes_remaining = available_bytes(offset, num_bytes);

		HAL_I2C_Mem_Write(I2C_handle, eeprom_address, mem_address, 2, &data[pos], bytes_remaining, 1000);

		pos = pos+bytes_remaining;

		offset = 0;

		start_page = start_page + 1;

		num_bytes = num_bytes - bytes_remaining;



	}


}


void eeprom_write(uint16_t page_num, uint8_t offset, uint8_t num_bytes, uint16_t *data){

	uint16_t left_shift = 6;

	uint16_t start_page = page_num;

	uint16_t end_page = start_page + ((offset+num_bytes)/bytes_per_page);

	uint16_t num_pages = (end_page - start_page) + 1;

	int pos = 0;

	while (start_page<num_pages){

		uint16_t mem_address = page_num<<left_shift;

		uint8_t bytes_remaining = available_bytes(offset, num_bytes);

		HAL_I2C_Mem_Read(I2C_handle, eeprom_address, mem_address, 2, &data[pos], bytes_remaining, 1000);

		pos = pos+bytes_remaining;

		offset = 0;

		start_page = start_page + 1;

		num_bytes = num_bytes - bytes_remaining;



	}


}
