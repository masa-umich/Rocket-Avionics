/**
 * Implementation file for communicating with the M24256E EEPROM chip.
 *
 * Author: Rohan Satapathy
 * Michigan Aeronautical Science Association
 */

#include "M24256E.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"
#include "string.h"

#define DEV_SELECT_MEM          0b1010
#define DEV_SELECT_ID_PAGE      0b1011
#define DEV_SELECT_ID_PAGE_LOCK 0b1011
#define DEV_SELECT_CDA          0b1011

#define I2C_ADDR_MEM(cda)     ((DEV_SELECT_MEM << 4) | (cda << 1))
#define I2C_ADDR_ID_PAGE(cda) ((DEV_SELECT_ID_PAGE << 4) | (cda << 1))
#define I2C_ADDR_ID_PAGE_LOCK(cda) \
    ((DEV_SELECT_ID_PAGE_LOCK << 4) | (cda << 1))
#define I2C_ADDR_CDA(cda) ((DEV_SELECT_CDA << 4) | (cda << 1))

#define B1ADDR_ID_PAGE      0b0 << 2
#define B1ADDR_ID_PAGE_LOCK 0b1 << 2
#define B1ADDR_CDA          0b110 << 5

#define B2ADDR_ID_PAGE_LOCK 0
#define B2ADDR_CDA          0

#define PAGE_SIZE      64
#define PAGE_ADDR_MASK 0b111111

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static inline void eeprom_enable_writes(eeprom_t* eeprom) {
    HAL_GPIO_WritePin(eeprom->wc.port, eeprom->wc.pin, GPIO_PIN_RESET);
}

static inline void eeprom_disable_writes(eeprom_t* eeprom) {
    HAL_GPIO_WritePin(eeprom->wc.port, eeprom->wc.pin, GPIO_PIN_SET);
}

/**
 * Perform a polling write to the EEPROM.
 *
 * If the M24256E is performing its internal write cycle, then any data
 * written to it will be responded to with a NACK. Therefore, when this
 * function receives a NACK, it re-transmits the data until it is
 * acknowledged.
 *
 * Datasheet: pg. 19
 *
 * @param eeprom      Struct used to store EEPROM I2C handle and pins.
 * @param dev_address The I2C address to use when writing.
 * @param data        The data to write to the EEPROM.
 * @param size        The number of bytes to write.
 * @param timeout     The HAL_I2C_Master_Transmit timeout (see HAL
 *                    documentation for more information).
 * @return            An eeprom_status_t indicating whether or not the write
 *                    was successful.
 */
static eeprom_status_t eeprom_polling_write(eeprom_t* eeprom,
                                            uint16_t dev_address,
                                            uint8_t* data, uint16_t size,
                                            uint32_t timeout) {
    HAL_StatusTypeDef ret;
    do {
        ret = HAL_I2C_Master_Transmit(eeprom->hi2c, dev_address, data, size,
                                      timeout);
    } while (ret != HAL_OK &&
             HAL_I2C_GetError(eeprom->hi2c) == HAL_I2C_ERROR_AF);

    // If the write failed for a reason that's not an acknowledgement
    // failure, then return an error.
    if (ret != HAL_OK) return EEPROM_TX_ERROR;
}

eeprom_status_t eeprom_init(eeprom_t* eeprom, I2C_HandleTypeDef* hi2c,
                            GPIO_TypeDef* wc_port, uint16_t wc_pin) {
    eeprom->hi2c = hi2c;
    eeprom->wc.port = wc_port;
    eeprom->wc.pin = wc_pin;
    eeprom->cda = 0b000;
    eeprom_disable_writes(eeprom);

    eeprom_status_t ret;
    uint8_t dest;
    for (uint8_t i = 0b000; i <= 0b111; i++) {
        eeprom->cda = i;
        ret = eeprom_read_cda(eeprom, &dest);
        if (ret == EEPROM_OK) return EEPROM_OK;
    }

    return EEPROM_INIT_ERROR;
}

eeprom_status_t eeprom_write_mem(eeprom_t* eeprom, uint16_t addr,
                                 uint8_t* data, uint16_t num_bytes) {
    // Check addr and num_bytes separately first to prevent overflow.
    if (addr > EEPROM_MEM_MAX_ADDR || num_bytes > EEPROM_MEM_MAX_ADDR ||
        addr + num_bytes > EEPROM_MEM_MAX_ADDR) {
        return EEPROM_INVALID_ARG;
    }

    eeprom_enable_writes(eeprom);

    uint8_t buf[sizeof(addr) + PAGE_SIZE];

    while (num_bytes > 0) {
        // Copy the address to the buffer
        memcpy(buf, &addr, sizeof(addr));

        // Copy the data chunk to the buffer. After the first loop
        // iteration, addr & PAGE_ADDR_MASK should be 0.
        uint8_t num_data_bytes_to_write =
            MIN(PAGE_SIZE, num_bytes) - (addr & PAGE_ADDR_MASK);
        memcpy(buf + sizeof(addr), data, num_data_bytes_to_write);

        eeprom_status_t ret = eeprom_polling_write(
            eeprom, I2C_ADDR_MEM(eeprom->cda), buf,
            sizeof(addr) + num_data_bytes_to_write, HAL_MAX_DELAY);
        if (ret != EEPROM_OK) {
            eeprom_disable_writes(eeprom);
            return ret;
        }

        // Increment counters
        data += num_data_bytes_to_write;
        addr += num_data_bytes_to_write;
        num_bytes -= num_data_bytes_to_write;
    }

    eeprom_disable_writes(eeprom);

    return EEPROM_OK;
}

eeprom_status_t eeprom_write_cda(eeprom_t* eeprom, uint8_t data) {
    if (data > 0b111) return EEPROM_INVALID_ARG;

    uint8_t buf[3];
    buf[0] = B1ADDR_CDA;
    buf[1] = B2ADDR_CDA;
    buf[2] = data;

    eeprom_enable_writes(eeprom);

    eeprom_status_t ret = eeprom_polling_write(
        eeprom, I2C_ADDR_CDA(eeprom->cda), buf, sizeof(buf), HAL_MAX_DELAY);
    if (ret != EEPROM_OK) {
        eeprom_disable_writes(eeprom);
        return ret;
    }

    eeprom_disable_writes(eeprom);

    eeprom->cda = data;
    return EEPROM_OK;
}

eeprom_status_t eeprom_write_id_page(eeprom_t* eeprom, uint8_t addr,
                                     uint8_t* data, uint8_t num_bytes) {
    // Check addr and num_bytes separately first to prevent overflow.
    if (addr > EEPROM_ID_PAGE_MAX_ADDR ||
        num_bytes > EEPROM_ID_PAGE_MAX_ADDR ||
        addr + num_bytes > EEPROM_ID_PAGE_MAX_ADDR) {
    }

    eeprom_enable_writes(eeprom);

    uint8_t buf[1 + sizeof(addr) + PAGE_SIZE];
    buf[0] = B1ADDR_ID_PAGE;
    buf[1] = addr;
    memcpy(buf + 1 + sizeof(addr), data, num_bytes);

    eeprom_status_t ret =
        eeprom_polling_write(eeprom, I2C_ADDR_ID_PAGE(eeprom->cda), buf,
                             1 + sizeof(addr) + num_bytes, HAL_MAX_DELAY);

    eeprom_disable_writes(eeprom);

    return ret;
}

eeprom_status_t eeprom_lock_id_page(eeprom_t* eeprom) {
    eeprom_enable_writes(eeprom);

    uint8_t buf[3];
    eeprom_status_t ret =
        eeprom_polling_write(eeprom, I2C_ADDR_ID_PAGE_LOCK(eeprom->cda),
                             buf, sizeof(buf), HAL_MAX_DELAY);

    eeprom_disable_writes(eeprom);

    return ret;
}

eeprom_status_t eeprom_read_cda(eeprom_t* eeprom, uint8_t* dest) {
    HAL_StatusTypeDef ret;
    uint8_t buf[2];

    // Initiate read
    buf[0] = B1ADDR_CDA;
    buf[1] = B2ADDR_CDA;
    ret = HAL_I2C_Master_Transmit(eeprom->hi2c, I2C_ADDR_CDA(eeprom->cda),
                                  buf, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return EEPROM_TX_ERROR;
    }

    // Perform read
    ret = HAL_I2C_Master_Receive(eeprom->hi2c, I2C_ADDR_CDA(eeprom->cda),
                                 buf, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return EEPROM_RX_ERROR;
    }

    *dest = buf[0];
    return EEPROM_OK;
}
