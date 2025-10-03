/*
 * MS5611_SIM.h
 *
 *  Created on: Oct 1, 2025
 *      Author: jackmh
 */

#ifndef MS5611_SIM_H
#define MS5611_SIM_H

#include "main.h"
#include <stdint.h> // make intellisense happy

// MS5611 commands (already defined elsewhere in your project? double check)
#define MS5611_CMD_RESET             0x1E
#define MS5611_CMD_ADC_READ          0x20
#define MS5611_CMD_PROM_READ_BASE    0xA0
#define MS5611_CMD_CONVERT_D1_BASE   0x40
#define MS5611_CMD_CONVERT_D2_BASE   0x50

#define UART_TRANSMIT_TIMEOUT_MS     100
#define UART_RECEIVE_TIMEOUT_MS      100

// Emulation state machine
typedef enum {
    WAITING_FOR_COMMAND,
    RESETTING,
    SENDING_PROM_DATA_LOW,
    SENDING_PROM_DATA_HIGH,
    D1_CONVERSION_REQUESTED,
    D2_CONVERSION_REQUESTED,
    SENDING_ADC_DATA_XL,
    SENDING_ADC_DATA_L,
    SENDING_ADC_DATA_H,
    SENDING_ADC_DATA_XH
} emulation_state_t;

// Function prototypes
void emulation_init();
void emulation_IRQHandler(SPI_HandleTypeDef *hspi);
void emulation_state_machine_update(uint8_t rx_byte);
uint8_t emulation_get_response(void);
void emulation_loop(UART_HandleTypeDef *huart);

#endif // MS5611_SIM_H
