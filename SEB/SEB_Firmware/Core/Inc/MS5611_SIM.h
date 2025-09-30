/*
 * MS5611_SIM.h
 *
 *  Created on: Jun 1, 2025
 *      Author: jackmh
 */

#ifndef MS5611_SIM_H
#define MS5611_SIM_H

#include "main.h"   // for HAL types like SPI_HandleTypeDef, UART_HandleTypeDef

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BUFFER_SIZE
#define BUFFER_SIZE 4   // Max buffer size for SPI packet, in bytes
#endif

// MS5611 command definitions
#define MS5611_CMD_RESET            0x1E
#define MS5611_CMD_ADC_READ         0x00
#define MS5611_CMD_PROM_READ_BASE   0xA0
#define MS5611_CMD_CONVERT_D1_BASE  0x40
#define MS5611_CMD_CONVERT_D2_BASE  0x50

// Timeouts
#define SPI_SLAVE_TRANSACTION_TIMEOUT_MS 100
#define UART_TRANSMIT_TIMEOUT_MS         100
#define UART_RECEIVE_TIMEOUT_MS          5000

// Extern variables
extern const uint16_t fixed_prom_values[8];

extern uint8_t SPI_rxBuffer[BUFFER_SIZE];
extern uint8_t SPI_txBuffer[BUFFER_SIZE];
extern uint8_t MS5611_ADC_Buffer[BUFFER_SIZE];
extern uint8_t UART_txBuffer[1];

// Function prototypes
void emulation_init(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart);

void emulation_callback(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* MS5611_SIM_H */
