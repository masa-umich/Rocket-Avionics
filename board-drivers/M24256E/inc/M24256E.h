#ifndef M24256E_H
#define M24256E_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIO_Pin_t;

typedef struct {
    I2C_HandleTypeDef* hi2c;
    GPIO_Pin_t WC;
} EEPROM_t;

typedef enum {
    EEPROM_OK = 0x00U,
    EEPROM_INVALID_ADDR = 0x01U,
    EEPROM_TX_ERROR = 0x02U,
    EEPROM_RX_ERROR = 0x03U,
} EEPROM_Status_t;

EEPROM_t EEPROM_init(I2C_HandleTypeDef* hi2c, GPIO_TypeDef* WC_port,
                     uint16_t WC_pin);

void EEPROM_writeMem(EEPROM_t* eeprom, uint16_t addr, uint8_t* bytes,
                     size_t numBytes);

void EEPROM_writeCDA(EEPROM_t* eeprom, uint8_t data);

void EEPROM_writeIDPage(EEPROM_t* eeprom, uint8_t addr, uint8_t* bytes,
                        size_t num_bytes);

void EEPROM_readMem(EEPROM_t* eeprom, uint16_t addr, size_t numBytes,
                    uint8_t* buf);

EEPROM_Status_t EEPROM_readCDA(EEPROM_t* eeprom, uint8_t* dest);

void EEPROM_readIDPage(EEPROM_t* eeprom, uint8_t addr, size_t numBytes,
                       uint8_t* buf);

bool EEPROM_isIDPageLocked(EEPROM_t* eeprom);

#endif  // !M24256E_H
