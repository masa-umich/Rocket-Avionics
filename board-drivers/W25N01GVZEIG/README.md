# W25N01GV Flash Memory User Guide (Last Updated: Nov 6 2020)
This repository contains the MASA firmware library for the W25N01GV flash memory IC.

[W25N01GV Datasheet](https://www.winbond.com/resource-files/w25n01gv%20revl%20050918%20unsecured.pdf)

[Application Note: Programming Winbond Flash Memory Devices](https://www.winbond.com/upload/technical-support/99ec7a3b-1b34-4206-b37e-58d8964695bf.pdf)

## Quick Start: Testing if Flash Works
1) Wire the W25N01GV flash chip with the WP and HLD pins set high.
2) Place `W25N01GV.h` in the `/Core/Inc` folder of your project, and place `W25N01GV.c` in the `/Core/Src` folder. Easiest way to do this is copy and paste (at least at this point in time). When it gets merged to the master branch, use the `git submodule` method described in the top-level directory's README.
3) `#include "W25N01GV.h"` in main.c
4) Initialize a `W25N01GV_Flash` struct and run the `is_flash_id_correct()` function, which returns `1` if SPI commands are correctly being sent and received.
5) Make sure SPI is at less than 100MHz, uses 8bit framing, MSB first, CPOL low, and CPHA 1 edge
```
#include "W25N01GV.h"
:
:
int main(void)
{
    :
    :
    W25N01GV_Flash flash;
    init_flash(&flash, &hspi2, GPIOB, GPIO_PIN_5);
    // Make sure you use the right spi bus and CS pin
    // For this example, GPIOB and GPIO_PIN_5 correspond to pin PB5 on the NucleoF446RE development board, which was used as the chip select (CS) pin.
    
    // This code turns the onboard LED (pin PA5) of the NucleoF446RE development board on when the flash chip is connected and communicating, and turns it off when it's not.
    while(1) {
        if (is_flash_id_correct(&flash))
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIO_A, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}
```
## SPI Configuration
The maximum SPI frequency allowed for this chip is 104 MHz. In the STM32CubeIde SPI configuration, the configuration settings should be identical to those below. Note: the prescaler is not noted below but should be adjusted accordingly

* Frame Format: Motorola
* Data Size: 8 bits
* First Bit: MSB First

Clock configuration:
* Clock Polarity: Low
* Clock Phase: 1 Edge

or
* Clock Polarity: High
* Clock Phase: 2 Edge

## Sample Code
### Initialization
Always run the `init_flash()` function first, using the correct `SPI_HandleTypeDef` struct, `GPIO_TypeDef` pin array, and `uint16_t` pin number.
The `GPIO_array` and `GPIO_pin` together correspond to pin `P<GPIO_array><GPIO_pin>` on the STM32 microcontroller. See the "Quick Start" section for an example.
```
W25N01GV_Flash flash;
init_flash(&flash, &<spi_bus_name>, <GPIO_array>, <GPIO_pin>);
```
### Reading from Flash
Read the entire flash memory array in 2KB chunks at a time.
NOTE: The total number of pages available to read is one greater than the maximum value of a `uint16_t`, so you either have to declare the page counter with at least 32bits or break out of the loop when the counter reaches `W25N01GV_NUM_PAGES`, or use some other control logic to avoid an infinite loop.
```
uint32_t page = 0;
uint8_t read_buffer[W25N01GV_BYTES_PER_PAGE];  // W25N01GV_BYTES_PER_PAGE == 2048 == 2KB
reset_flash_read_pointer(&flash);

while (page < flash.current_page) {
    read_next_2KB_from_flash(&flash, read_buffer);
    // Data gets read into read_buffer, do something with it here
}
```
TODO: include code on how to ignore last page if the last page is completely empty (see pressurization board firmware)

### Writing to Flash
Write an array of `uint8_t` bytes to flash. Note that the minimum amount of data that can be reliably written to flash is 512B, so data is stored in a temporary write buffer in the `W25N01GV_Flash` struct, which is only sent to flash once it fills up.
`TODO:` include a README section about the exact contents of the W25N01GV_Flash struct.
NOTE: If there isn't enough space to write the data, anything over capacity will get cut off and won't be written.
```
// data is a uint8_t array, num_bytes is the number of elements in data
write_to_flash(&flash, data, num_bytes);
:
:
:
:
finish_flash_write(&flash);
```
At the end of your program, you MUST call `finish_flash_write()` to write the remaining contents of the write buffer to flash. This must be done before the W25N01GV_Flash struct goes out of scope, because it contains the array with leftover data. If you don't do this, you will lose up to the last 512B of data passed to `write_to_flash()`.
TODO: add more details about when to use finish_flash_write and how it affects the data formatting
### Checking SPI Functionality
You can check if you can successfully send and receive data to flash over the SPI bus by using `is_flash_id_correct()` to "ping" flash. This sample turns an LED on if it's successful, and turns it off if it's not. The GPIO pin array and number used here are the ones for the green onboard LED on the STM32F446RE Nucleo board.
```
while (1) {
    if (is_flash_id_correct(&flash)) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}
```
### Checking for Bad Memory Blocks
Up to 20 out of the 1024 memory blocks are allowed to be defective when shipped. This is marked at the first byte of every defective block, so you need to scan each block before erasing and overwriting data. To do this, use the function `scan_bad_blocks()` after initialization. If `scan_bad_blocks()` returns a nonzero value, there is at least 1 corrupted memory block and you should consider using a different chip.
```
uint16_t bad_block_addresses[1024];
uint16_t num_bad_blocks = scan_bad_blocks(&flash, bad_block_addresses);
```

## Reading/Writing to Reserved Pages
This firmware implements a pseudo-EEPROM functionality by reserving the last block (64 pages/128 KB) to be modified directly by the user.
### Writing/Erasing
`write_reserved_flash_page()` takes in a flash struct pointer, a page address from 0 to 63, inclusive, a `uint8_t` array of up to 2048 items, and the size of the array. It will write the array to the specified page in the last block of flash, starting at the first byte in the page. The same restrictions apply here as for writing to flash normally, that is, you can't edit data after it's been written, and you can only write to a page that has previously been erased.

`erase_reserved_flash_pages()` takes in only the flash struct pointer. It will erase all 64 reserved pages, so make sure that when you are only editing one value, save all of the data written to those pages, edit what you need to, erase, then write it all back.
Note: `erase_flash()` will not erase these 64 pages.

```
// This example writes ambients to the first reserved page, and PID gains to the second flash page, after first erasing the reserved pages. 
(I have no idea if these numbers are realistic, doesn't really matter though)

uint8_t ambient_data[6] = { 0x01, 0x34, 0x45, 0x67, 0x89, 0x00 };
uint8_t pid_gains[12] = { 0x00, 0x00, 0x00 0x56, 0x00, 0x00, 0xFD, 0x4A, 0x00, 0x00, 0x00 0x9E };

erase_reserved_flash_pages(&flash);
write_reserved_flash_pages(&flash, 0, ambient_data, 6);
write_reserved_flash_pages(&flash, 1, pid_gains, 12);
```
Alternatively, you could pack all of the data you want (ambients, pid gains, setpoints, etc) into 1 array and only write to page 0, as long as it's 2048 bytes or shorter.
### Reading
`read_reserved_flash_page()` takes in the exact same parameters as `write_reserved_flash_page()`, except data from the specified page will get read into the array. This example shows the data from the previous example getting read.

```
uint8_t read_ambients[6];
uint8_t read_pid_gains[12];

read_reserved_flash_page(&flash, 0, read_ambients, 6);
read_reserved_flash_page(&flash, 1, read_pid_gains, 12);

// User will have to unpack the bytes into the appropriate variables
```
