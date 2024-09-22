/**
 * Implementation MASA W25N01GV Flash Memory firmware library
 * Datasheet: https://www.winbond.com/resource-files/w25n01gv%20revl%20050918%20unsecured.pdf
 * 
 * Nathaniel Kalantar (nkalan@umich.edu)
 * Michigan Aeronautical Science Association
 * Created July 20, 2020
 * Last edited February 26, 2021
 *
 * This code assumes the WP and HLD pins are always set high.
 *
 * ===============================================================================
 * Memory architecture described on datasheet pg 11
 * Device operational flow described on datasheet pg 12
 *
 * Nathaniel Kalantar (nkalan@umich.edu)
 * Michigan Aeronautical Science Association
 * Created July 20, 2020
 * Last edited November 6, 2020
 */

#include "../inc/W25N01GV.h"

#ifdef HAL_SPI_MODULE_ENABLED  // Begin SPI include protection

// Device ID information, used to check if flash is working
#define	W25N01GV_MANUFACTURER_ID                  (uint8_t)  0xEF
#define W25N01GV_DEVICE_ID                        (uint16_t) 0xAA21

// Chip is active low
#define W25N01GV_CS_ACTIVE                        (uint8_t)  GPIO_PIN_RESET
#define W25N01GV_CS_INACTIVE                      (uint8_t)  GPIO_PIN_SET

// Arbitrary timeout value
#define W25N01GV_SPI_TIMEOUT                      (uint8_t)  0xFF

// 1024 blocks with 64 pages each = 65536 pages
#define W25N01GV_PAGES_PER_BLOCK                  (uint16_t) 64
#define W25N01GV_NUM_BLOCKS                       (uint16_t) 1024

// Used for find_file_ptr()
#define W25N01GV_ERASED_BYTE                               (uint8_t) 0xFF

/* Commands */
// Summary of commands and usage on datasheet pg 23-25
#define W25N01GV_DEVICE_RESET                     (uint8_t) 0xFF
#define W25N01GV_READ_JEDEC_ID                    (uint8_t) 0x9F
#define W25N01GV_READ_STATUS_REGISTER             (uint8_t) 0x0F  // 0x05 also works
#define W25N01GV_WRITE_STATUS_REGISTER            (uint8_t) 0x1F  // 0x01 also works
#define W25N01GV_WRITE_ENABLE                     (uint8_t) 0x06
#define W25N01GV_WRITE_DISABLE                    (uint8_t) 0x04
#define W25N01GV_READ_BBM_LOOK_UP_TABLE           (uint8_t) 0xA5
#define W25N01GV_ERASE_BLOCK                      (uint8_t) 0xD8
#define W25N01GV_LOAD_PROGRAM_DATA                (uint8_t) 0x02
#define W25N01GV_PROGRAM_EXECUTE                  (uint8_t) 0x10
#define W25N01GV_PAGE_DATA_READ                   (uint8_t) 0x13
#define W25N01GV_READ_DATA                        (uint8_t) 0x03

/* Status Register addressses */
#define W25N01GV_SR1_PROTECTION_REG_ADR           (uint8_t) 0xA0  // Listed as 0xAx in the datasheet
#define W25N01GV_SR2_CONFIG_REG_ADR               (uint8_t) 0xB0  // Listed as 0xBx in the datasheet
#define W25N01GV_SR3_STATUS_REG_ADR               (uint8_t) 0xC0  // Listed as 0xBx in the datasheet

/* Status Register bits */
// Protection register - datasheet pg 15
#define W25N01GV_SR1_BLOCK_PROTECT_BP3            (uint8_t) 0x40  // 0b01000000
#define W25N01GV_SR1_BLOCK_PROTECT_BP2            (uint8_t) 0x20  // 0b00100000
#define W25N01GV_SR1_BLOCK_PROTECT_BP1            (uint8_t) 0x10  // 0b00010000
#define W25N01GV_SR1_BLOCK_PROTECT_BP0            (uint8_t) 0x08  // 0b00001000
#define W25N01GV_SR1_BLOCK_PROTECT_TB             (uint8_t) 0x04  // 0b00000100
#define W25N01GV_SR1_WRITE_PROTECT_ENABLE         (uint8_t) 0x02  // 0b00000010
#define W25N01GV_SR1_STATUS_REGISTER_PROTECT_SRP1 (uint8_t) 0x00  // 0b10000000
#define W25N01GV_SR1_STATUS_REGISTER_PROTECT_SRP0 (uint8_t) 0x01  // 0b00000001

// Configuration register - datasheet pg 17
#define W25N01GV_SR2_ONE_TIME_PROGRAM_LOCK        (uint8_t) 0x80  // 0b10000000
#define W25N01GV_SR2_ENTER_TOP_ACCESS_MODE        (uint8_t) 0x40  // 0b01000000
#define W25N01GV_SR2_STATUS_REGISTER_1_LOCK       (uint8_t) 0x20  // 0b00100000
#define W25N01GV_SR2_ECC_ENABLE                   (uint8_t) 0x10  // 0b00010000
#define W25N01GV_SR2_BUFFER_READ_MODE             (uint8_t) 0x08  // 0b00001000

// Status register - datasheet pg 19
#define W25N01GV_SR3_BBM_LOOK_UP_TABLE_FULL       (uint8_t) 0x40  // 0b01000000
#define W25N01GV_SR3_ECC_STATUS_BIT_1             (uint8_t) 0x20  // 0b00100000
#define W25N01GV_SR3_ECC_STATUS_BIT_0             (uint8_t) 0x10  // 0b00010000
#define W25N01GV_SR3_PROGRAM_FAILURE              (uint8_t) 0x08  // 0b00001000
#define W25N01GV_SR3_ERASE_FAILURE                (uint8_t) 0x04  // 0b00000100
#define W25N01GV_SR3_WRITE_ENABLE_LATCH           (uint8_t) 0x02  // 0b00000010
#define W25N01GV_SR3_OPERATION_IN_PROGRESS        (uint8_t) 0x01  // 0b00000001

// Flash internal delays - datasheet pg 59
// Used for firmware timeouts
#define W25N01GV_RESET_MAX_TIME_US                    500  // datasheet pg 26
#define W25N01GV_WRITE_STATUS_REGISTER_TIME_NS         50
#define W25N01GV_BLOCK_ERASE_MAX_TIME_MS                4
#define W25N01GV_READ_PAGE_DATA_ECC_ON_MAX_TIME_US     25
#define W25N01GV_READ_PAGE_DATA_ECC_OFF_MAX_TIME_US    60
#define W25N01GV_PAGE_PROGRAM_MAX_TIME_US             700


/* Private functions */

/**
 * Converts a uint16_t number into an array literal of two uint8_t numbers.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param num        <uint16_t>           A 16 bit number to be converted to an array of two 8 bit numbers.
 * @retval An array literal of 2 uint8_t numbers, with the first entry
 * 	representing the first 8 bits	of the input, and the second entry
 * 	representing the last 8 bits of the input.
 */
#define W25N01GV_UNPACK_UINT16_TO_2_BYTES(num)		{(uint8_t) (((num) & 0xFF00) >> 8), (uint8_t) ((num) & 0x00FF)}

/**
 * Converts an array of 2 uint8_t numbers into 1 uint16_t number.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param bytes      <uint8_t*>           A pointer to an array of two uint8_t numbers
 * @retval A uint16_t equal to (2^8)*B1 + B0, where bytes = {B1, B0}, which is
 *   equivalent to concatenating B1B0.
 */
#define W25N01GV_PACK_2_BYTES_TO_UINT16(bytes)		(uint16_t) ((((uint16_t) *(bytes)) << 8) + *((bytes)+1))

/**
 * Transmit 1 or more bytes to the device via SPI. This function exists to
 * shorten the number of commands required to write something over SPI
 * to 2 lines of code: define tx, then call this function.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param tx         <uint8_t*>           Data buffer to transmit
 * @param size       <uint16_t>           Number of bytes to transmit
 */
static void spi_transmit(W25N01GV_Flash *flash, uint8_t *tx, uint16_t size) {

	__disable_irq();
	HAL_GPIO_WritePin(flash->cs_base, flash->cs_pin, W25N01GV_CS_ACTIVE);  // Select chip
	// Transmit data and store the status code
	flash->last_HAL_status = HAL_SPI_Transmit(flash->SPI_bus, tx, size, W25N01GV_SPI_TIMEOUT);
	HAL_GPIO_WritePin(flash->cs_base, flash->cs_pin, W25N01GV_CS_INACTIVE);  // Release chip
	__enable_irq();

}

/**
 * Transmit 1 or more bytes and receive 1 or more bytes to/from the device via SPI.
 * This function exists to shorten the number of commands required to write
 * and read something over SPI to 3 lines of code: define tx array, declare rx, and then
 * call this function.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param tx         <uint8_t*>           Data buffer to transmit
 * @param tx_size    <uint16_t>           Number of bytes to transmit
 * @param rx         <uint8_t*>           Buffer to receive data
 * @param rx_size    <uint16_t>           Number of bytes to receive
 */
static void spi_transmit_receive(W25N01GV_Flash *flash, uint8_t *tx,
		uint16_t tx_size,	uint8_t *rx, uint16_t rx_size) {

	__disable_irq();
	HAL_GPIO_WritePin(flash->cs_base, flash->cs_pin, W25N01GV_CS_ACTIVE);  // Select chip
	// Transmit/receive, and store the status code
	flash->last_HAL_status = HAL_SPI_Transmit(flash->SPI_bus, tx, tx_size, W25N01GV_SPI_TIMEOUT);
	flash->last_HAL_status = HAL_SPI_Receive(flash->SPI_bus, rx, rx_size, W25N01GV_SPI_TIMEOUT);
	// TODO the Transmit status will get lost, should it still be stored like this?
	HAL_GPIO_WritePin(flash->cs_base, flash->cs_pin, W25N01GV_CS_INACTIVE);  // Release chip
	__enable_irq();

}

/**
 * The Read Status Register instruction may be used at any time, even while
 * a Program or Erase cycle is in progress.
 *
 * The status registers available to read are:
 * Protection Register (SR1)				Address: W25N01GV_SR1_PROTECTION_REG_ADR (0xA0)
 * Configuration Register (SR2)			Address: W25N01GV_SR2_CONFIG_REG_ADR (0xB0)
 * Status Register (SR3)						Address: W25N01GV_SR3_STATUS_REG_ADR (0xC0)
 *
 * datasheet pg 28
 *
 * @param flash        <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param register_adr <uint8_t>            The address of the status register to be read
 * @retval The contents of the 8 bit status register specified by the user
 */
static uint8_t read_status_register(W25N01GV_Flash *flash, uint8_t register_adr) {
	uint8_t tx[2] = {W25N01GV_READ_STATUS_REGISTER, register_adr};
	uint8_t rx[1];

	spi_transmit_receive(flash, tx, 2, rx, 1);

	return *rx;
}

/**
 * Checks to see if flash is busy. While it's busy, all commands will be
 * ignored except for Read Status Register and Read JEDEC ID.
 *
 * BUSY is a read only bit in the status register (S0) that is set to a 1
 * state when the device is powering up or executing a Page Data Read, Bad
 * Block Management, Program Execute, Block Erase, Program Execute for OTP
 * area, OTP Locking or after a Continuous Read instruction.
 *
 * datasheet pg 20
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @retval 0 if the device is busy,  nonzero integer (1) if it's not.
 *
 * TODO: make sure if flash is broken/not plugged in it doens't return the wrong value/cause infinite loops
 */
static uint8_t flash_is_busy(W25N01GV_Flash *flash) {
	uint8_t status_register = read_status_register(flash, W25N01GV_SR3_STATUS_REG_ADR);
	return status_register & W25N01GV_SR3_OPERATION_IN_PROGRESS;
}

/**
 * Pings flash with the flash_is_busy() function to check if it's
 * currently busy with an operation. It stays in this function waiting
 * for the operation to finish when flash_is_busy() returns 0 or until
 * it reaches the timeout.
 *
 * Note: this function should assume a 180 MHz maximum clock frequency, which
 * corresponds to a minimum 5.5555 ns period.
 *
 * TODO (low priority): Change function to accurately count the times using timers,
 * right now it's a duct tape solution
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param timeout    <uint32_t>           Maximum time to wait in nanoseconds.
 */
static void wait_for_operation(W25N01GV_Flash *flash, uint32_t timeout) {
	uint32_t count = 0;
	while (flash_is_busy(flash) && count < 6*timeout) {
		++count;
	}
}

/**
 * Writes 1 byte to the selected status register. Takes at least 50 ns to complete.
 *
 * The status registers available to write are:
 * Protection Register (SR1)				Address: W25N01GV_SR1_PROTECTION_REG_ADR (0xA0)
 * Configuration Register (SR2)			Address: W25N01GV_SR2_CONFIG_REG_ADR (0xB0)
 *
 * Includes additional delay of 50 ns
 *
 * datasheet pg 29
 *
 * @param flash              <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param register_adr       <uint8_t>            The address of the status register to be written to
 * @param register_write_val <uint8_t>            The 8bit value to write to the status register
 * 	Note: this will overwrite all values in the register
 */
static void write_status_register(W25N01GV_Flash *flash, uint8_t register_adr,
		uint8_t register_write_val) {
	uint8_t tx[3] = {W25N01GV_WRITE_STATUS_REGISTER, register_adr, register_write_val};

	spi_transmit(flash, tx, 3);

	wait_for_operation(flash, W25N01GV_WRITE_STATUS_REGISTER_TIME_NS);
}

/**
 * Checks to see if all 20 entries in the Bad Block Management look up table are full.
 *
 * datasheet pg 19, 32
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @retval 0 if LUT is not full, nonzero int (64) if full
 */
static uint8_t BBM_look_up_table_is_full(W25N01GV_Flash *flash) {
	uint8_t status_register = read_status_register(flash, W25N01GV_SR3_STATUS_REG_ADR);
	return status_register & W25N01GV_SR3_BBM_LOOK_UP_TABLE_FULL;
}

/**
 * Read the bad block management look up table. The manufacturer marks some of
 * the bad memory blocks and writes them into a look up table. The datasheet
 * says to scan all blocks before writing or erasing on a new flash chip
 * so you don't delete the bad block information that's loaded at the factory.
 *
 * TODO: maybe shorten the parameter names
 *
 * datasheet pg 32
 *
 * @param flash                    <W25N01GV_Flash*> Struct used to store flash pins and addresses
 * @param logical_block_addresses  <uint16_t*>       Pointer to array of 20 uint16_t's to store the table's LBAs
 * @param physical_block_addresses <uint16_t*>       Pointer to array of 20 uint16_t's to store the table's PBAs
 */
static void read_BBM_look_up_table(W25N01GV_Flash *flash, uint16_t *logical_block_addresses,
		uint16_t *physical_block_addresses) {
	uint8_t tx[2] = {W25N01GV_READ_BBM_LOOK_UP_TABLE, 0};	 // 2nd byte is unused
	uint8_t rx[80];

	spi_transmit_receive(flash, tx, 2, rx, 80);

	// Format the received bytes into the user-supplied arrays
	for (int i = 0; i < 20; i++) {
		logical_block_addresses[i] = W25N01GV_PACK_2_BYTES_TO_UINT16(rx+(2*i));
		physical_block_addresses[i] = W25N01GV_PACK_2_BYTES_TO_UINT16(rx+(2*i)+1);
	}
}

/**
 * Loads a page specified by the user into the device's buffer.
 * Load process takes 25 microseconds if ECC is disabled, and 60 microseconds if enabled.
 * The device will be in a BUSY state and ignore most commands until loading finishes.
 *
 * datasheet pg 38
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param page_num   <uint16_t>           Page number of data to load to the device's buffer
 */
static void load_page(W25N01GV_Flash *flash, uint16_t page_num) {
	uint8_t page_num_8bit_array[2] = W25N01GV_UNPACK_UINT16_TO_2_BYTES(page_num);
	uint8_t tx[4] = {W25N01GV_PAGE_DATA_READ, 0, page_num_8bit_array[0], page_num_8bit_array[1]};  // 2nd byte is unused

	spi_transmit(flash, tx, 4);

	// TODO currently assumes ECC is always on, but needs to be more flexible
  wait_for_operation(flash, W25N01GV_READ_PAGE_DATA_ECC_ON_MAX_TIME_US * 1000);  // Wait for the page to load
}

/**
 * Unlocks write capabilities on the device. All memory arrays are locked to
 * read-only when the device is powered on to protect data. This function
 * unlocks memory by changing the block protect bits in the Protection register
 * to all 0's. It does not change the other bits in the register.
 *
 * datasheet pg 15, 21
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
static void unlock_flash(W25N01GV_Flash *flash) {
	// Read the current contents of the protection register
	uint8_t protection_register = read_status_register(flash, W25N01GV_SR1_PROTECTION_REG_ADR);

	// Remove the block protect bits, and only keep the 3 non-protect bits, if they're already enabled.
	uint8_t unlocked_protection_register = protection_register & ~(W25N01GV_SR1_BLOCK_PROTECT_BP3
			| W25N01GV_SR1_BLOCK_PROTECT_BP2
			| W25N01GV_SR1_BLOCK_PROTECT_BP1
			| W25N01GV_SR1_BLOCK_PROTECT_BP0
			| W25N01GV_SR1_BLOCK_PROTECT_TB);

	// Write the new value of the status register block protect bits off
	write_status_register(flash, W25N01GV_SR1_PROTECTION_REG_ADR, unlocked_protection_register);
}

/**
 * Locks write capabilities on the device. This function unlocks memory by
 * changing the block protect bits in the Protection register to a locked
 * configuration. It does not change the other bits in the register.
 *
 * fix this function - first remove the bits, then add them back on
 * TODO implemented, now test this fix
 *
 * datasheet pg 15, 21
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
static void lock_flash(W25N01GV_Flash *flash) {
	// Read the current contents of the protection register
	uint8_t protection_register = read_status_register(flash, W25N01GV_SR1_PROTECTION_REG_ADR);

	// Enabling bits BP3 and BP2 and disabling the others will lock the entire 128MB memory array
	// First remove all protection bits
	uint8_t locked_protection_register = protection_register & ~(W25N01GV_SR1_BLOCK_PROTECT_BP3
			| W25N01GV_SR1_BLOCK_PROTECT_BP2
			| W25N01GV_SR1_BLOCK_PROTECT_BP1
			| W25N01GV_SR1_BLOCK_PROTECT_BP0
			| W25N01GV_SR1_BLOCK_PROTECT_TB);

	// Then add on the bits that are needed to lock flash
	locked_protection_register = protection_register
			| (W25N01GV_SR1_BLOCK_PROTECT_BP3 | W25N01GV_SR1_BLOCK_PROTECT_BP2);

	write_status_register(flash, W25N01GV_SR1_PROTECTION_REG_ADR, locked_protection_register);
}

/**
 * Enables writing to flash by sending a command to set the
 * Write Enable Latch (WEL) bit in the status register to 1.
 *
 * The WEL bit is automatically reset after Power-up and upon completion of
 * the Page Program, Quad Page Program, Block Erase, Reset and Bad Block
 * Management instructions.
 *
 * datasheet pg 30
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
static void enable_write(W25N01GV_Flash *flash) {
	uint8_t tx[1] = { W25N01GV_WRITE_ENABLE };
	spi_transmit(flash, tx, 1);
}

/**
 * Disables writing to flash by sending a command to set the
 * Write Enable Latch (WEL) bit in the status register to 0.
 *
 * The WEL bit is automatically reset after Power-up and upon completion of
 * the Page Program, Quad Page Program, Block Erase, Reset and Bad Block
 * Management instructions.
 *
 * datasheet pg 30
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
static void disable_write(W25N01GV_Flash *flash) {
	uint8_t tx[1] = { W25N01GV_WRITE_DISABLE };
	spi_transmit(flash, tx, 1);
}

/**
 * Writes data to the device's buffer in preparation for writing
 * it to memory. It writes the data at the specified column (byte), and writes
 * data up to column 2047 or the end of the user supplied data array.
 *
 * datasheet pg 35
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param data       <uint8_t*>           Data array containing data to write to flash
 * @param num_bytes  <uint16_t>           Number of bytes to write
 * @param column_adr <uint16_t>           Byte in buffer to start writing at
 */
static void write_page_to_buffer(W25N01GV_Flash *flash, uint8_t *data,
		uint16_t num_bytes, uint16_t column_adr) {

	uint8_t column_adr_8bit_array[2] = W25N01GV_UNPACK_UINT16_TO_2_BYTES(column_adr);
	uint8_t tx1[3] = {W25N01GV_LOAD_PROGRAM_DATA, column_adr_8bit_array[0], column_adr_8bit_array[1]};

	// Ignore all data that would be written to column 2048 and after.
	// You don't want to overwrite the extra memory at the end of the page.
	// (If the onboard ECC is turned on, this happens automatically, but just in case)
	if (num_bytes > W25N01GV_BYTES_PER_PAGE)
		num_bytes = W25N01GV_BYTES_PER_PAGE;

	// Not using spi_transmit() because I didn't want to mess with combining the tx arrays
	__disable_irq();
	HAL_GPIO_WritePin(flash->cs_base, flash->cs_pin, W25N01GV_CS_ACTIVE);
	flash->last_HAL_status = HAL_SPI_Transmit(flash->SPI_bus, tx1, 3, W25N01GV_SPI_TIMEOUT);
	flash->last_HAL_status = HAL_SPI_Transmit(flash->SPI_bus, data, num_bytes, W25N01GV_SPI_TIMEOUT);
	HAL_GPIO_WritePin(flash->cs_base, flash->cs_pin, W25N01GV_CS_INACTIVE);
	__enable_irq();
}

/**
 * Run the program execute command to store the data in the device's buffer into
 * memory at the specified page. This should be run after running write_page_to_buffer().
 *
 * This function incurs a typical delay of 250 microseconds,
 * with a max delay of 700 microseconds.
 *
 * datasheet pg 37
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param page_adr   <uint16_t>           The page for the buffer to be written to.
 */
static void program_buffer_to_memory(W25N01GV_Flash *flash, uint16_t page_adr) {
	uint8_t page_adr_8bit_array[2] = W25N01GV_UNPACK_UINT16_TO_2_BYTES(page_adr);
	uint8_t tx[4] = {W25N01GV_PROGRAM_EXECUTE, 0, page_adr_8bit_array[0], page_adr_8bit_array[1]};  // 2nd byte unused

	spi_transmit(flash, tx, 4);
	wait_for_operation(flash, W25N01GV_PAGE_PROGRAM_MAX_TIME_US * 1000);	 // Wait for the data to be written to memory
}

/**
 * Checks whether or not the last program write command executed successfully
 * by reading the status register (SR3) and checking the program failure bit,
 * then stores that bit in the W25N01GV_Flash struct.
 *
 * This should always return 1 if the last page written to is in a
 * protected part of the memory array (for this firmware, when flash is
 * locked), or if the write enable command is not given before writing.
 * In both cases, flash's memory array at that page shouldn't be changed.
 *
 * datasheet pg 20
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @retval 0x08 if it detects a write failure, 0 if no write failures were detected
 */
static uint8_t get_write_failure_status(W25N01GV_Flash *flash) {
	// If it can't read from flash, it will automatically return a write failure
	if (ping_flash(flash)) {
		uint8_t status_register = read_status_register(flash, W25N01GV_SR3_STATUS_REG_ADR);
		flash->last_write_failure_status = status_register & W25N01GV_SR3_PROGRAM_FAILURE;
	}
	else {
		flash->last_write_failure_status = W25N01GV_SR3_PROGRAM_FAILURE;
	}

	return flash->last_write_failure_status;
}

/**
 * Checks whether or not the last erase command executed successfully
 * by reading the status register (SR3) and checking the erase failure bit,
 * then stores that bit in the W25N01GV_Flash struct.
 *
 * This should always return 1 if the last page written to is in a
 * protected part of the memory array (for this firmware, when flash is
 * locked), or if the write enable command is not given before writing.
 * In both cases, flash's memory array at that page shouldn't be changed.
 *
 * datasheet pg 20
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @retval 0x04 if it detects an erase failure, 0 if no erase failures were detected
 */
static uint8_t get_erase_failure_status(W25N01GV_Flash *flash) {
	// If it can't read from flash, it will automatically return an erase failure
	if (ping_flash(flash)) {
		uint8_t status_register = read_status_register(flash, W25N01GV_SR3_STATUS_REG_ADR);
		flash->last_erase_failure_status = status_register & W25N01GV_SR3_ERASE_FAILURE;
	}
	else {
		flash->last_erase_failure_status = W25N01GV_SR3_ERASE_FAILURE;
	}

	return flash->last_erase_failure_status;

}

/**
 * Erases all data in the block containing the specified page address.
 * Each block has 64 pages, for a total of 128KB. Data is erased by setting
 * each byte to 0xFF.
 *
 * The Write Enable Latch bit is first set high, and is automatically set low
 * after the erase process finishes.
 *
 * If the block containing the specified page address is protected by the
 * Block Protect bits in the protection register, then the erase command
 * will not execute.
 *
 * It takes up to 10 milliseconds to complete the process, but typically takes
 * 2 milliseconds (datasheet pg 59).
 *
 * datasheet pg 34
 *
 * Note: the input parameter is the address of a page in the block you want to
 * erase (between 0 and W25N01GV_NUM_PAGES-1), not the block number (0 to 1023).
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param page_adr   <uint16_t>           Address of the page whose block should be erased
 */
static void erase_block(W25N01GV_Flash *flash, uint16_t page_adr) {
	enable_write(flash);	// Set WEL bit high, it will automatically be set back to 0 after the command executes

	uint8_t page_adr_8bit_array[2] = W25N01GV_UNPACK_UINT16_TO_2_BYTES(page_adr);
	uint8_t tx[4] = {W25N01GV_ERASE_BLOCK, 0, page_adr_8bit_array[0], page_adr_8bit_array[1]};	// 2nd byte unused
	spi_transmit(flash, tx, 4);

	disable_write(flash);	// Disable WEL just in case the erase block command doens't execute

	wait_for_operation(flash, W25N01GV_BLOCK_ERASE_MAX_TIME_MS * 1000000);  // Wait for it to finish erasing

	get_erase_failure_status(flash);
}

/**
 * Reads the status of the error corrections done on the last read command.
 * This function should be used after read operations to verify data integrity.
 * The ECC status is stored in the flash struct.
 *
 * It reads the ECC1 and ECC0 bits of the status register (SR3) and determines
 * what the error status is, based on the table in the datasheet.
 *
 * datasheet pg 20
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
static void get_ECC_status(W25N01GV_Flash *flash) {

	// If it can read from flash properly, check the ECC bits as normal
	if (ping_flash(flash)) {
		uint8_t status_register, ECC1, ECC0;

		status_register = read_status_register(flash, W25N01GV_SR3_STATUS_REG_ADR);
		ECC1 = status_register & W25N01GV_SR3_ECC_STATUS_BIT_1;
		ECC0 = status_register & W25N01GV_SR3_ECC_STATUS_BIT_0;

		// Return status according to table on datasheet pg 20
		if (!ECC1 && !ECC0)
			flash->last_read_ECC_status = SUCCESS_NO_CORRECTIONS;
		else if (!ECC1 && ECC0)
			flash->last_read_ECC_status = SUCCESS_WITH_CORRECTIONS;
		else if (ECC1 && !ECC0)
			flash->last_read_ECC_status = ERROR_ONE_PAGE;
		else  // else if (ECC1 && ECC0)
			flash->last_read_ECC_status = ERROR_MULTIPLE_PAGES;

	}
	else {  // Otherwise record the read error
		flash->last_read_ECC_status = READ_ERROR_NO_ECC_STATUS;
	}
}

/**
 * Reads the contents of the flash's buffer into an array, starting at the
 * specified column and going until it reaches the end of the buffer or
 * reads in buffer_size number of bytes.
 *
 * This function incurs a delay that varies linearly with num_bytes
 * and the SPI clock period.
 *
 * This function is called by read_bytes_from_page().
 *
 * datasheet pg 39
 *
 * @param flash       <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param buffer      <uint8_t*>           Array to read the contents of the device's buffer into
 * @param num_bytes   <uint16_t>           Number of bytes to read into buffer
 * @param column_adr  <uint16_t>           Starting column address of the data to be read in
 */
static void read_flash_buffer(W25N01GV_Flash *flash, uint8_t *buffer,
		uint16_t num_bytes, uint16_t column_adr) {

	uint8_t column_adr_8bit_array[2] = W25N01GV_UNPACK_UINT16_TO_2_BYTES(column_adr);
	uint8_t tx[4] = {W25N01GV_READ_DATA, column_adr_8bit_array[0], column_adr_8bit_array[1], 0};  // last byte is unused

	spi_transmit_receive(flash, tx, 4, buffer, num_bytes);
}

/**
 * Reads the specified number of uint8_t bytes from the
 * specified page and column into an array.
 *
 * It should only be used with buffer mode enabled [buffer mode is enabled
 * by default both during power-on and reset, and in init_flash()]
 *
 * This function incurs a delay that varies linearly with num_bytes
 * and the SPI clock period.
 *
 * Note: if num_bytes > (2112 - column_adr), then it will only
 * read in (2112 - column_adr) bytes
 *
 * TODO: add datasheet pages
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param buffer     <uint8_t*>           Data buffer to read data into
 * @param num_bytes  <uint16_t>           Number of bytes to read in (see note on range)
 * @param page_adr   <uint16_t>           The page to read data from
 * @param column_adr <uint16_t>           The column to start reading data from
 */
static void read_bytes_from_page(W25N01GV_Flash *flash, uint8_t *buffer, uint16_t num_bytes,
		uint16_t page_adr, uint16_t column_adr) {

	load_page(flash, page_adr);  // Load the page into flash's buffer
	read_flash_buffer(flash, buffer, num_bytes, column_adr);

	get_ECC_status(flash);
}

/**
 * Writes the contents of data into flash at the specified page and column.
 * It writes to the device's buffer, then programs the buffer data into flash memory.
 *
 * It then reads the write failure status bit and stores it to the W25N01GV_Flash struct.
 *
 * This function incurs a delay that varies linearly with num_bytes
 * and the SPI clock period.  // TODO is this note necessary?
 *
 * Note: unlock_flash() must be called before calling this function,
 * otherwise it will do nothing.
 *
 * TODO: add datasheet pages
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 * @param num_bytes  <uint16_t>           Number of bytes to write to flash
 * @param page_adr   <uint16_t>           Page to write data to
 * @param column_adr <uint16_t>           Column of page to start writing data at
 */
static void write_bytes_to_page(W25N01GV_Flash *flash, uint8_t *data, uint16_t num_bytes,
		uint16_t page_adr, uint16_t column_adr) {

	enable_write(flash);

	write_page_to_buffer(flash, data, num_bytes, column_adr);
	program_buffer_to_memory(flash, page_adr);

	// This will happen automatically if program_buffer_to_memory() succeeds, but just in case it fails ;)
	disable_write(flash);

	get_write_failure_status(flash);
}

/**
 * Set the ECC-E bit in the configuration register (SR2) to 1, enabling the
 * onboard error correction algorithms. If ECC-E is already 1, does nothing.
 *
 * datasheet pg 18
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
static void enable_ECC(W25N01GV_Flash *flash) {
	uint8_t config_reg_read = read_status_register(flash, W25N01GV_SR2_CONFIG_REG_ADR);
	uint8_t ECC_enabled_register = config_reg_read | W25N01GV_SR2_ECC_ENABLE;	 // OR: turn bit on

	if (ECC_enabled_register != config_reg_read)
		write_status_register(flash, W25N01GV_SR2_CONFIG_REG_ADR, ECC_enabled_register);
}

/**
 * Set the ECC-E bit in the configuration register (SR2) to 0, disabling the
 * onboard error correction algorithms. If ECC-E is already 0, does nothing.
 * Don't actually use this, but I included it because why not.
 *
 * datasheet pg 18
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
static void disable_ECC(W25N01GV_Flash *flash) {
	uint8_t config_reg_read = read_status_register(flash, W25N01GV_SR2_CONFIG_REG_ADR);
	uint8_t ECC_disabled_register = config_reg_read & ~W25N01GV_SR2_ECC_ENABLE;	 // Remove bit

	if (ECC_disabled_register != config_reg_read)
		write_status_register(flash, W25N01GV_SR2_CONFIG_REG_ADR, ECC_disabled_register);
}

/**
 * Sets the device to buffer read mode, which limits the user to reading
 * up to one page at a time before loading a new page. It sets the BUF bit
 * in the configuration register to 1 if BUF=0, and does nothing if BUF=1.
 * This is the mode that MASA firmware will be using for TSM.
 *
 * datasheet pg 18
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
static void enable_buffer_mode(W25N01GV_Flash *flash) {
	uint8_t config_reg_read = read_status_register(flash, W25N01GV_SR2_CONFIG_REG_ADR);
	uint8_t buffer_enabled_register = config_reg_read | W25N01GV_SR2_BUFFER_READ_MODE;	// OR: turn bit on
	if (buffer_enabled_register != config_reg_read)
		write_status_register(flash, W25N01GV_SR2_CONFIG_REG_ADR, buffer_enabled_register);
}

/**
 * Performs a binary search on flash memory to find the first available
 * address to write to. Modifies flash->current_page and flash->next_free_column.
 *
 * This function is critical to prevent data corruption, which occurs when this firmware
 * tries to write over previously-written addresses.
 *
 * Key assumptions made by this function that are maintained by write_to_flash():
 * All non-empty pages come before all empty pages in memory. This in turn assumes
 * that the user is not constantly writing '0xFF', which is the empty/erased byte.
 * If there is entire page consisting of '0xFF', and there are later pages that
 * contain user-written data, then this function might fail.
 * TODO: write a note in write_to_flash() not to do this
 *
 * Another way to think of this function is that there should always be a contiguous chunk
 * of 0xFF bytes at the end of flash, and this function finds the first byte in that chunk.
 *
 * Expected results:
 * flash->current_page will be any uint16_t,and flash->next_free_column will be
 * 0, 512, 1024, or 1536 (except when flash->current_page == 65536, in which case
 * flash->next_free_column can also be 2048).
 * See various comments about flash->write_buffer for explanation.
 *
 * @param flash      <W25N01GV_Flash*>    Struct used to store flash pins and addresses
 */
static void find_write_ptr(W25N01GV_Flash *flash) {
	uint8_t read_buffer[2048];

	// First check page 0 for if flash has already been erased.
	// Checking this case first because it's probably pretty common.
	read_bytes_from_page(flash, read_buffer, 2048, 0, 0);
	uint8_t first_page_empty = 1;
	for (uint16_t b = 0; b < 2048; b++) {
		if (read_buffer[b] != 0xFF) {
			first_page_empty = 0;
		}
	}
	if (first_page_empty) {
		flash->current_page = 0;
		flash->next_free_column = 0;
		return;
	}

	// Binary search on all of flash to find the last page written to.

	// min and max declared as 32bit because W25N01GV_NUM_PAGES > largest uint16_t
	uint32_t min = 0;  // inclusive
	uint32_t max = W25N01GV_NUM_PAGES;  // exclusive
	uint16_t cur_search_page;

	while (max - min > 1) {  // Keep looping until you narrow range down a single page
		cur_search_page = min + (max-min) / 2;

		// Read cur_search_page and check if it's empty
		read_bytes_from_page(flash, read_buffer, 2048, cur_search_page, 0);
		uint8_t cur_page_empty = 1;
		for (uint16_t b = 0; b < 2048; b++) {
			if (read_buffer[b] != 0xFF) {
				cur_page_empty = 0;
				break;
			}
		}

		if (cur_page_empty)  // Found an empty page - move to the left sector
			max = cur_search_page;
		else  // Found a non-empty page - move to the right sector
			min = cur_search_page;  // Don't completely exclude it from range
	}
	// Breaks out of the loop when range is narrowed down to [min, min+1),
	flash->current_page = min;

	// After finding flash->current_page, do a linear search on that page
	// to find flash->next_free_address
	read_bytes_from_page(flash, read_buffer, 2048, flash->current_page, 0);

	// Edge case: if the page found by the binary search is completely full
	// (last byte is non-0xFF) then the write pointer should actually be the
	// first byte of the next page (breaking out of the while loop guaranteed
	// that page min+1 is empty).	If that is the last page in flash, handle
	// accordingly to make get_bytes_remaining() return 0.

	if (read_buffer[2047] != 0xFF) {  // aforementioned edge case
		if (flash->current_page == W25N01GV_NUM_PAGES-1) {  // no room left in flash
			flash->next_free_column = 2048;
		}
		else {  // go to start of next page
			flash->current_page++;
			flash->next_free_column = 0;
		}

		return;
	}
	else {  // normal linear search
		// Start at the end and search backwards for the first non-empty byte
		uint8_t page_empty = 1;
		for (int16_t b = 2047; b >= 0; b--) {
			if (read_buffer[b] != 0xFF) {
				flash->next_free_column = b+1;
				page_empty = 0;
				break;
			}
		}

		// Debug code
		// TODO: delete in final release
		/*
		if (page_empty) {
			asm("nop");  // if you get here, I fucked up
		}
		*/
	}

	// flash->next_free_column should only ever be at the beginning of one of the sectors on a page.
	// If the user's last byte(s) written are 0xFF, then the previous loop will treat it as empty memory.
	// This if block maintains the 512 byte framing.
	// At this point in the code, flash->current_page is not empty and flash is not completely full.
	if (flash->next_free_column <= 512)
		flash->next_free_column = 512;
	else if (flash->next_free_column <= 1024)
		flash->next_free_column = 1024;
	else if (flash->next_free_column <= 1536)
		flash->next_free_column = 1536;
	else if (flash->next_free_column < 2048) {  // Increment to next page
		flash->next_free_column = 0;
		flash->current_page++;
	}
}


/* Public function definitions */

void init_flash(W25N01GV_Flash *flash, SPI_HandleTypeDef *SPI_bus_in,
		GPIO_TypeDef *cs_base_in,	uint16_t cs_pin_in) {
	flash->SPI_bus = SPI_bus_in;
	flash->cs_base = cs_base_in;
	flash->cs_pin = cs_pin_in;
	flash->next_page_to_read = 0;

	flash->last_HAL_status = HAL_OK;
	flash->last_read_ECC_status = SUCCESS_NO_CORRECTIONS;
	flash->last_write_failure_status = 0;
	flash->last_erase_failure_status = 0;

	reset_flash(flash);

	enable_ECC(flash);  // Should be enabled by default, but enable ECC just in case
	enable_buffer_mode(flash);  // -IG models start with buffer mode by default, -IT models don't
	// As of the time of writing this, MASA uses the -IG model.

	find_write_ptr(flash);
}

uint8_t ping_flash(W25N01GV_Flash *flash) {

	uint8_t tx[2] = {W25N01GV_READ_JEDEC_ID, 0};	// Second byte is unused
	uint8_t rx[3];

	spi_transmit_receive(flash, tx, 2, rx, 3);
	uint8_t manufacturer_ID = rx[0];
	uint16_t device_ID = W25N01GV_PACK_2_BYTES_TO_UINT16(rx+1);

	if (manufacturer_ID == W25N01GV_MANUFACTURER_ID && device_ID == W25N01GV_DEVICE_ID)
		return 1;
	else
		return 0;
}

uint8_t reset_flash(W25N01GV_Flash *flash) {
	// The reset command will corrupt data if given while another
	// operation is taking place, so just return in that case
	if (flash_is_busy(flash))  // TODO: replace with wait_for_operation? idk honestly
		return 0;

	// Otherwise send the reset command
	uint8_t tx[1] = { W25N01GV_DEVICE_RESET };
	spi_transmit(flash, tx, 1);

	// Wait for it to reset
	wait_for_operation(flash, W25N01GV_RESET_MAX_TIME_US * 1000);

	return 1;
}

/**
 * Old write function that writes everything immediately.
 * The reason why this is separate from write_flash() is because I wrote
 * this function before I learned about the 512-byte framing requirement,
 * so write_to_flash() handles the framing and uses this function to do
 * the actual writing.
 *
 * Read the application note linked in the README to read about the
 * 512-byte framing requirement and why violating it will corrupt data.
 *
 * ASSUMPTIONS:
 * flash->next_free_column is a multiple of W25N01GV_SECTOR_SIZE between [0, 2047].
 * data can fit in flash's remaining space.
 * flash->write_buffer is not full == flash->write_buffer_size < W25N01GV_SECTOR_SIZE.
 * Flash is unlocked.
 *
 */
static uint16_t write_to_flash_contiguous(W25N01GV_Flash *flash, uint8_t *data, uint32_t num_bytes) {

	// Debug code
	// TODO: delete in final release
	/*
	if (!(flash->next_free_column == 0 || flash->next_free_column == 512
			|| flash->next_free_column == 1024 || flash->next_free_column == 1536
			|| flash->next_free_column == 2048)) {
		asm("nop");
	}
	*/

	// Debug code
	// TODO: delete in final release
	/*
	if (num_bytes % 512 != 0) {
		asm("nop");
	}
	*/

	uint32_t write_counter = 0;  // Track how many bytes have been written so far
	uint16_t write_failures = 0;  // Track write errors

	while (write_counter < num_bytes) {

		// If there's not enough space on the page, only write as much as will fit
		uint16_t num_bytes_to_write_on_page = num_bytes - write_counter;
		if (num_bytes_to_write_on_page > W25N01GV_BYTES_PER_PAGE - flash->next_free_column)
			num_bytes_to_write_on_page = W25N01GV_BYTES_PER_PAGE - flash->next_free_column;

		// Write the array (or a part of it if it's too long for one page) to flash
		write_bytes_to_page(flash, data + write_counter, num_bytes_to_write_on_page,
				flash->current_page, flash->next_free_column);

		// Check if the page was written to correctly
		if (flash->last_write_failure_status)
			write_failures++;

		write_counter += num_bytes_to_write_on_page;

		// If there's room left over at the end of the page,
		// increment the column counter and leave the page counter the same
		if (flash->next_free_column + num_bytes_to_write_on_page < W25N01GV_BYTES_PER_PAGE)
			flash->next_free_column += num_bytes_to_write_on_page;

		// If it fills the current page and runs out of pages, set the column counter over
		// the limit so it can't write again (will make get_bytes_remaining() return 0)
		else if (flash->current_page == W25N01GV_NUM_PAGES-1)
			flash->next_free_column = W25N01GV_BYTES_PER_PAGE;

		// Otherwise if there's more pages left, bring the address counter to the next page
		// and reset the column counter
		else {
			flash->next_free_column = 0;
			flash->current_page++;  // This function can assume it won't run out of pages
		}
	}

	// Debug code
	// TODO: delete in final release
	/*
	if (!(flash->next_free_column == 0 || flash->next_free_column == 512
			|| flash->next_free_column == 1024 || flash->next_free_column == 1536
			|| flash->next_free_column == 2048)) {
		asm("nop");
	}
	*/

	return write_failures;
}

uint16_t write_to_flash(W25N01GV_Flash *flash, uint8_t *data, uint32_t num_bytes) {

	// If there's not enough space, truncate the data
	uint32_t bytes_remaining = get_bytes_remaining(flash);
	if (num_bytes > bytes_remaining)
		num_bytes = bytes_remaining;

	uint16_t write_failures = 0;  // Track write failures

	// Copy the front end into the write_buffer
	uint8_t buffer_full = 0;  // Used to decide whether or not to write buffer contents to flash

	// If write_buffer is not empty and data fills it completely
	if (flash->write_buffer_size > 0 && flash->write_buffer_size + num_bytes >= W25N01GV_SECTOR_SIZE) {
		// Copy data into write_buffer until it's full,
		uint16_t num_bytes_to_copy = W25N01GV_SECTOR_SIZE - flash->write_buffer_size;
		for (uint16_t i = 0; i < num_bytes_to_copy; ++i) {
			flash->write_buffer[flash->write_buffer_size + i] = data[i];
		}
		flash->write_buffer_size = W25N01GV_SECTOR_SIZE;
		buffer_full = 1;

		// Adjust data and num_bytes to reflect the front end being chopped off
		data += num_bytes_to_copy;
		num_bytes -= num_bytes_to_copy;
	}
	// If data doesn't fill write_buffer completely
	else if (flash->write_buffer_size + num_bytes < W25N01GV_SECTOR_SIZE) {
		// Copy all data into write_buffer and return
		for (uint16_t i = 0; i < num_bytes; ++i) {
			flash->write_buffer[flash->write_buffer_size + i] = data[i];
		}
		flash->write_buffer_size += num_bytes;

		return 0;
	}

	// In the case where write_buffer is empty and data would fill it completely,
	// write_buffer isn't used and the write is handled separately.

	// At this point, data and num_bytes should be adjusted so it starts at a multiple address.
	// Use integer division to find out where the last unit is.
	// If there are less than W25N01GV_SECTOR_SIZE, then end_size == num_bytes and end_arr == data,
	// so anything in data that doesn't get written in this function will get stored in write_buffer
	uint32_t new_data_size = (num_bytes / W25N01GV_SECTOR_SIZE) * W25N01GV_SECTOR_SIZE;
	uint16_t end_size = num_bytes % W25N01GV_SECTOR_SIZE;
	uint8_t* end_arr = data + new_data_size;

	unlock_flash(flash);

	// If the buffer got filled, write the buffer to flash using write_to_flash_contiguous()
	if (buffer_full) {
		write_failures += write_to_flash_contiguous(flash, flash->write_buffer, W25N01GV_SECTOR_SIZE);
		flash->write_buffer_size = 0;
	}

	// Write the processed array into flash using write_to_flash_contiguous()
	if (new_data_size > 0) {
		write_failures += write_to_flash_contiguous(flash, data, new_data_size);
	}

	lock_flash(flash);

	// Copy the remaining data, if any, into write_buffer
	for (uint16_t i = 0; i < end_size; ++i) {
		flash->write_buffer[i] = end_arr[i];
	}
	flash->write_buffer_size = end_size;

	return write_failures;

}

uint16_t finish_flash_write(W25N01GV_Flash *flash) {
	// Ignore this function if there's nothing in the write buffer
	if (flash->write_buffer_size == 0) {
		return 0;
	}

	// Fill the rest of write_buffer with 0x00 to prevent
	// any future accidental calls to write_to_flash() don't
	// mess up the 512-byte framing
	while (flash->write_buffer_size < W25N01GV_SECTOR_SIZE)
		flash->write_buffer[flash->write_buffer_size++] = 0x00;

	// If there's not enough space, truncate the data.
	// This should never happen, but just in case.
	uint32_t bytes_remaining = get_bytes_remaining(flash);
	if (flash->write_buffer_size > bytes_remaining)
		flash->write_buffer_size = bytes_remaining;

	unlock_flash(flash);

	uint16_t write_failures = write_to_flash_contiguous(flash, flash->write_buffer,
			flash->write_buffer_size);
	flash->write_buffer_size = 0;

	lock_flash(flash);

	return write_failures;
}

void reset_flash_read_pointer(W25N01GV_Flash *flash) {
	flash->next_page_to_read = 0;
}

void read_next_2KB_from_flash(W25N01GV_Flash *flash, uint8_t *buffer) {
	read_bytes_from_page(flash, buffer,	W25N01GV_BYTES_PER_PAGE, flash->next_page_to_read, 0);
	flash->next_page_to_read++;  // Increment the page read counter

	get_ECC_status(flash);
}

uint16_t erase_flash(W25N01GV_Flash *flash) {
	uint16_t erase_failures = 0;

	unlock_flash(flash);

	// Loop through every block to erase them one by one
	// Ignore the last block, which is reserved for pseudo-eeprom functionality
	for (uint16_t block_count = 0; block_count < W25N01GV_NUM_BLOCKS-1; block_count++) {
		erase_block(flash, block_count * W25N01GV_PAGES_PER_BLOCK);  // Address of first page in each block

		// Check if the erase failed
		if (flash->last_erase_failure_status)
			erase_failures++;
	}

	lock_flash(flash);

	// Reset the address pointer after erasing
	find_write_ptr(flash);  // Don't manually set addr pointers to ensure it actually erases
	flash->write_buffer_size = 0;

	return erase_failures;
}

uint32_t get_bytes_remaining(W25N01GV_Flash *flash) {
	return ((W25N01GV_NUM_BLOCKS-1) * W25N01GV_PAGES_PER_BLOCK * W25N01GV_BYTES_PER_PAGE)
			- (flash->current_page * W25N01GV_BYTES_PER_PAGE + flash->next_free_column)
			- flash->write_buffer_size;

	// write_buffer hasn't been written to flash yet, but its size needs to be counted
	// to get an accurate count for the user.
}

uint8_t write_reserved_flash_page(W25N01GV_Flash *flash, uint8_t page_num, uint8_t* data, uint16_t data_sz) {
	// Write to the nth page of the last block of flash
	unlock_flash(flash);
	write_bytes_to_page(flash, data, data_sz,
			(W25N01GV_NUM_BLOCKS-1) * W25N01GV_PAGES_PER_BLOCK + page_num, 0);
	lock_flash(flash);

	return flash->last_write_failure_status;
}

void read_reserved_flash_page(W25N01GV_Flash *flash, uint8_t page_num, uint8_t* buffer, uint16_t buffer_sz) {
	// Grab the nth page of the last block of flash
	read_bytes_from_page(flash, buffer, buffer_sz,
			(W25N01GV_NUM_BLOCKS-1) * W25N01GV_PAGES_PER_BLOCK + page_num, 0);
}

uint8_t erase_reserved_flash_pages(W25N01GV_Flash *flash) {
	// Erase the last block only
	unlock_flash(flash);
	erase_block(flash, W25N01GV_PAGES_PER_BLOCK * (W25N01GV_NUM_BLOCKS - 1));
	lock_flash(flash);
	return flash->last_erase_failure_status;
}

uint16_t scan_bad_blocks(W25N01GV_Flash *flash, uint16_t *bad_blocks) {

	uint8_t read_byte[1];
	uint16_t num_bad_blocks = 0;

	for (uint16_t block_adr = 0; block_adr < W25N01GV_NUM_BLOCKS; block_adr++) {  // block 0, 1, ..., 1022, 1023
		read_bytes_from_page(flash, read_byte, 1, block_adr*W25N01GV_PAGES_PER_BLOCK, 0);  // page 0, 64, 128, ...

		if (*read_byte != 0xFF) {  // Look for non-0xFF bytes
			bad_blocks[num_bad_blocks] = block_adr;
			num_bad_blocks++;
		}
	}

	return num_bad_blocks;
}

void add_test_delimiter(W25N01GV_Flash *flash) {
	// This is kind of dumb but it works
	uint8_t delimiter_arr[W25N01GV_BYTES_PER_PAGE] = { 0 };

	// Fill an entire page worth of bytes with 0's
	write_to_flash(flash, delimiter_arr, W25N01GV_BYTES_PER_PAGE);
}

#endif	// End SPI include protection
