/**
 * Header file for communicating with the M24256E EEPROM chip.
 * Datasheet: https://www.st.com/resource/en/datasheet/m24256e-f.pdf
 *
 * Author: Rohan Satapathy
 * Michigan Aeronautical Science Association
 */

#ifndef M24256E_H
#define M24256E_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#define EEPROM_MEM_MAX_ADDR     0x7FFF
#define EEPROM_ID_PAGE_MAX_ADDR 0x3F

/**
 * gpio_pin_t stores a combination of an STM32 port and pin number for
 * convenience.
 */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} gpio_pin_t;

/**
 * eeprom_t stores a pointer to the I2C handle and the WC pin information.
 * A pointer to a struct of this type is passed to each function in this
 * driver.
 */
typedef struct {
    // The STM32 HAL I2C handle.
    I2C_HandleTypeDef* hi2c;

    // The write control pin (active-low).
    gpio_pin_t wc;

    // The contents of the CDA register.
    uint8_t cda;
} eeprom_t;

/**
 * eeprom_status_t enumerates the possible failure modes of functions in
 * this driver. Every function returns an eeprom_status_t, and it is
 * strongly recommended that calling code explicitly check the return value
 * of each function called.
 */
typedef enum {
    EEPROM_OK,
    EEPROM_INVALID_ARG,
    EEPROM_TX_ERROR,
    EEPROM_RX_ERROR,
    EEPROM_INIT_ERROR,
} eeprom_status_t;

/**
 * Initialize the eeprom_t struct.
 *
 * In addition to storing the I2C handle and write control (WC) port/pin
 * information, this function determines the contents of the M24256E CDA
 * register. The 2nd, 3rd, and 4th least significant bits of this register
 * are used to determine the EEPROM's I2C address, so determining the
 * contents of the CDA register is necessary for all communication with the
 * EEPROM (see Datasheet pg. 15 for more information).
 *
 * @param eeprom  The uninitialized eeprom_t struct.
 * @param hi2c    The I2C handle, initialized by the STM32CubeIDE generated
 *                code.
 * @param wc_port The STM32 port where the write control (WC) pin is
 *                connected.
 * @param wc_pin  The pin number where the write control (WC) pin is
 *                connected.
 * @return        An eeprom_status_t indicating whether or not the
 *                initialization is successful. This function will only
 *                fail if the contents of the CDA register is unable to
 *                be determined.
 */
eeprom_status_t eeprom_init(eeprom_t* eeprom, I2C_HandleTypeDef* hi2c,
                            GPIO_TypeDef* wc_port, uint16_t wc_pin);

/**
 * Write data to the EEPROM starting at addr.
 *
 * The M24256E has 512 64-byte pages and supports writing to one page per
 * write cycle. If data needs to be written across a page boundary, this
 * function will initiate multiple write cycles until the data has been
 * completely written.
 *
 * If any of the data to be written extends past the last valid address
 * (i.e. if addr + num_bytes - 1 > EEPROM_MEM_MAX_ADDR), this function will
 * NOT write any data and return EEPROM_INVALID_ARG.
 *
 * Datasheet: pp. 13-14
 *
 * @param eeprom    Struct used to store EEPROM I2C handle and pins.
 * @param addr      The address at which to store the first byte in the
 *                  buffer.
 * @param data      A pointer to the data to write to the EEPROM.
 * @param num_bytes The number of bytes to write to the EEPROM.
 * @return          An eeprom_status_t indicating whether or not the write
 *                  was successful.
 */
eeprom_status_t eeprom_write_mem(eeprom_t* eeprom, uint16_t addr,
                                 uint8_t* data, uint16_t num_bytes);

/**
 * Write to the EEPROM's configurable device address (CDA) register.
 *
 * The M24256E uses the 2nd, 3rd, and 4th least significant bits of the CDA
 * register to form the I2C address used to read from/write to the EEPROM.
 * If the least significant bit is ever written HIGH, the CDA register will
 * be locked and cannot be written to in the future. For this reason, this
 * function will always left-shift data by 1 to ensure that the least
 * significant bit is always zero.
 *
 * Datasheet: pg. 15
 *
 * @param eeprom Struct used to store EEPROM I2C handle and pins.
 * @param data   The value to write to the CDA register. If 0b000 <= data <=
 *               0b111 is not true, then EEPROM_INVALID_ARG will be
 *               returned.
 * @return       An eeprom_status_t indicating whether or not the write was
 *               successful.
 */
eeprom_status_t eeprom_write_cda(eeprom_t* eeprom, uint8_t data);

/**
 * Write data from a buffer to the EEPROM's ID page.
 *
 * If the data to be written extends past the page boundary (i.e. if addr +
 * num_bytes - 1 > EEPROM_ID_PAGE_MAX_ADDR), this function will NOT write
 * any data and return EEPROM_INVALID_ARG.
 *
 * Datasheet: pg. 16
 *
 * @param eeprom    Struct used to store EEPROM I2C handle and pins.
 * @param addr      The address at which to write the first byte of data.
 * @param data      A pointer to the data buffer.
 * @param num_bytes The number of bytes to write.
 * @return          An eeprom_status_t indicating whether or not the write
 *                  was successful.
 */
eeprom_status_t eeprom_write_id_page(eeprom_t* eeprom, uint8_t addr,
                                     uint8_t* data, uint8_t num_bytes);

/**
 * Permanently lock the EEPROM's ID page.
 *
 * WARNING: This will disable all future writes to the ID page. Forever. For
 * all time. No takesies backsies.
 *
 * Datasheet: pg. 18
 *
 * @param eeprom Struct used to store EEPROM I2C handle and pins.
 * @return       An eeprom_status_t indicating whether or not the lock
 *               operation was successful.
 */
eeprom_status_t eeprom_lock_id_page(eeprom_t* eeprom);

/**
 * Read data from the EEPROM starting at addr into dest.
 *
 * If any of the data to be read is at an invalid address (i.e. if addr +
 * num_bytes - 1 > EEPROM_MEM_MAX_ADDR), then data will NOT be read and
 * EEPROM_INVALID_ARG will be returned.
 *
 * Datashet: pp. 21-22
 *
 * @param eeprom    Struct used to store EEPROM I2C handle and pins.
 * @param addr      The address of the first byte of data to read.
 * @param dest      A pointer to the buffer in which to store the read data.
 *                  It is the caller's responsibility to ensure that this
 *                  buffer is initialized with a size of at least num_bytes.
 * @param num_bytes The number of bytes to read from the EEPROM.
 * @return          An eeprom_status_t indicating whether or not the read
 *                  was successful.
 */
eeprom_status_t eeprom_read_mem(eeprom_t* eeprom, uint16_t addr,
                                uint8_t* dest, uint16_t num_bytes);

/**
 * Read the contents of the CDA register.
 *
 * NOTE: This function does not simply return eeprom->cda, it performs an
 * I2C read of the CDA register.
 *
 * Datasheet: pg. 23
 *
 * @param eeprom Struct used to store EEPROM I2C handle and pins.
 * @param dest   A pointer to the location to store the contents of the CDA
 *               register.
 * @return       An eeprom_status_t indicating whether or not the read was
 *               successful.
 */
eeprom_status_t eeprom_read_cda(eeprom_t* eeprom, uint8_t* dest);

/**
 * Read data from the EEPROM ID page starting at addr into buf.
 *
 * If any of the data to be read is at an invalid address (i.e. if addr +
 * num_bytes - 1 > EEPROM_ID_PAGE_MAX_ADDR), then data will NOT be read and
 * EEPROM_INVALID_ARG will be returned.
 *
 * Datasheet: pg. 24
 *
 * @param eeprom    Struct used to store EEPROM I2C handle and pins.
 * @param addr      The address of the first byte of data to read.
 * @param dest      A pointer to the buffer in which to store the data. It
 *                  is the caller's responsibility to ensure that this
 *                  buffer is initialized with a size of at least num_bytes.
 * @param num_bytes The number of bytes to read.
 * @return          An eeprom_status_t indicating whether or not the read
 *                  was successful.
 */
eeprom_status_t eeprom_read_id_page(eeprom_t* eeprom, uint8_t addr,
                                    uint8_t* dest, uint8_t num_bytes);

/**
 * Set dest to true if the EEPROM ID page is locked.
 *
 * Datasheet: pg. 25
 *
 * @param eeprom Struct used to store EEPROM I2C handle and pins.
 * @param dest   A pointer to the bool where the result is stored.
 * @return       An eeprom_status_t indicating whether or not the read was
 *               successful.
 */
eeprom_status_t eeprom_is_id_page_locked(eeprom_t* eeprom, bool* dest);

#endif  // !M24256E_H
