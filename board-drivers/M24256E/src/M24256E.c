#include "M24256E.h"

#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"

static inline void EEPROM_enableWrites(EEPROM_t* eeprom) {
    HAL_GPIO_WritePin(eeprom->WC.port, eeprom->WC.pin, GPIO_PIN_RESET);
}

static inline void EEPROM_disableWrites(EEPROM_t* eeprom) {
    HAL_GPIO_WritePin(eeprom->WC.port, eeprom->WC.pin, GPIO_PIN_SET);
}

EEPROM_t EEPROM_init(I2C_HandleTypeDef* hi2c, GPIO_TypeDef* WC_port,
                     uint16_t WC_pin) {
    EEPROM_t eeprom = {.hi2c = hi2c, .WC = {.port = WC_port, .pin = WC_pin}};
    EEPROM_disableWrites(&eeprom);
    return eeprom;
}

EEPROM_Status_t EEPROM_readCDA(EEPROM_t* eeprom, uint8_t* dest) {
    HAL_StatusTypeDef ret;
    uint8_t buf[2];

    // Initiate read
    buf[0] = EEPROM_B1ADDR_CDA;
    buf[1] = EEPROM_B2ADDR_CDA;
    ret = HAL_I2C_Master_Transmit(eeprom->hi2c, EEPROM_DEV_SELECT_CDA, buf, 2,
                                  HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return EEPROM_TX_ERROR;
    }

    // Perform read
    ret = HAL_I2C_Master_Receive(eeprom->hi2c, EEPROM_DEV_SELECT_CDA, buf, 1,
                                 HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        return EEPROM_RX_ERROR;
    }

    *dest = buf[0];
    return EEPROM_OK;
}
