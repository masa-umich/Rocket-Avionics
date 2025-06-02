/*
 * MS5611_SIM.c
 *
 *  Created on: Jun 1, 2025
 *      Author: jackmh
 */

#include "main.h"

#ifndef BUFFER_SIZE
#define BUFFER_SIZE 4 // Max buffer size for SPI packet, in bytes
#endif

#define MS5611_CMD_RESET            0x1E // Reset command
#define MS5611_CMD_ADC_READ         0x00 // ADC read command
#define MS5611_CMD_PROM_READ_BASE   0xA0 // PROM Read commands are 0xA0 to 0xAE. Base is 0xA0. Address is in bits 1,2,3 of the command.=
#define MS5611_CMD_CONVERT_D1_BASE  0x40 // Convert D1 (pressure) commands are 0x40 to 0x48. Base is 0x40.
#define MS5611_CMD_CONVERT_D2_BASE  0x50 // Convert D2 (temperature) commands are 0x50 to 0x58. Base is 0x50.

#define SPI_SLAVE_TRANSACTION_TIMEOUT_MS 100 // Timeout for SPI transmit operations
#define UART_TRANSMIT_TIMEOUT_MS         100 // Timeout for UART transmit operations
#define UART_RECEIVE_TIMEOUT_MS          5000 // 5 seconds, adjust as needed

const uint16_t fixed_prom_values[8] = {
    0xABCD, // Addr 0: doesn't matter filler value
    40127,  // Addr 1: C1 (Pressure Sensitivity SENS_T1)
    36924,  // Addr 2: C2 (Pressure Offset OFF_T1)
    23317,  // Addr 3: C3 (Temp. coeff. of pressure sensitivity TCS)
    23282,  // Addr 4: C4 (Temp. coeff. of pressure offset TCO)
    33464,  // Addr 5: C5 (Reference temperature T_REF)
    28312,  // Addr 6: C6 (Temp. coeff. of the temperature TEMPSENS)
    0x1234  // Addr 7: CRC but we dont use it
};

extern uint8_t SPI_rxBuffer[BUFFER_SIZE]; // Extern because it needs to global
extern uint8_t SPI_txBuffer[BUFFER_SIZE]; 

uint8_t MS5611_ADC_Buffer[BUFFER_SIZE] = {0}; // Buffer for ADC read command response
uint8_t UART_txBuffer[1] = {0}; // UART transmit buffer for D1 or D2 conversion commands    

void emulation_loop(SPI_HandleTypeDef *hspi, UART_HandleTypeDef *huart) {
    HAL_StatusTypeDef spi_status;
    HAL_StatusTypeDef uart_status;
    uint8_t SPI_command;

	while (1) {
        spi_status = HAL_SPI_Receive(hspi, SPI_rxBuffer, 1, HAL_MAX_DELAY);
        if (spi_status == HAL_OK) {
        	SPI_command = SPI_rxBuffer[0];

            // This would be a switch statement but because of the way we're doing bit manipulation it's easier to do this with ifs
            if (SPI_command == MS5611_CMD_RESET) {
                // Pretend to reset or something
            } else if ((SPI_command & 0xF0) == MS5611_CMD_PROM_READ_BASE) {
                // PROM Read Command
                uint8_t prom_address_index = (SPI_command & 0x0E) >> 1;

                if (prom_address_index < 8) { // Check for valid PROM address index (0-7)
                    // Prepare the 2-byte (16-bit) PROM value to be sent.
                    // MSB first, then LSB.
                    SPI_txBuffer[0] = (fixed_prom_values[prom_address_index] >> 8) & 0xFF; // MSB
                    SPI_txBuffer[1] = fixed_prom_values[prom_address_index] & 0xFF;       // LSB
                    
                    // Transmit the 2 bytes back to the SPI master.
                    // The master will clock these bytes out during the same CS assertion.
                    HAL_SPI_Transmit(hspi, SPI_txBuffer, 2, SPI_SLAVE_TRANSACTION_TIMEOUT_MS);
                } else {
                    // Invalid PROM address requested by master.
                    // Send dummy data (e.g., 0xFFFF) as a response.
                    SPI_txBuffer[0] = 0xFF;
                    SPI_txBuffer[1] = 0xFF;
                    HAL_SPI_Transmit(hspi, SPI_txBuffer, 2, SPI_SLAVE_TRANSACTION_TIMEOUT_MS);
                }
            } else if (((SPI_command & 0xF0) == MS5611_CMD_CONVERT_D1_BASE) ||
                       ((SPI_command & 0xF0) == MS5611_CMD_CONVERT_D2_BASE)) {
                       // Conversion Command for D1 (Pressure) or D2 (Temperature)

                UART_txBuffer[0] = SPI_command; // Store the command to be sent over UART
                // Transmit the command over UART
                uart_status = HAL_UART_Transmit(huart, UART_txBuffer, 1, UART_TRANSMIT_TIMEOUT_MS);
                if (uart_status != HAL_OK) {
                    // Failed transmit
                    // Error_Handler(); // Handle the error as needed
                }
                uart_status = HAL_UART_Receive(huart, MS5611_ADC_Buffer, 3, UART_RECEIVE_TIMEOUT_MS);
                if (uart_status != HAL_OK) {
                    // No response
                    // Error_Handler(); // Handle the error as needed
                }   
            } else if (SPI_command == MS5611_CMD_ADC_READ) {
                // ADC Read command (0x00)
                // Transmit the 3-byte simulated ADC value (previously received from UART and stored).
                // The master will clock these bytes out during the same CS assertion.
                // `SPI_txBuffer` (global) is used here.
            	SPI_txBuffer[0] = 0xFE;
            	SPI_txBuffer[1] = MS5611_ADC_Buffer[0];
                SPI_txBuffer[2] = MS5611_ADC_Buffer[1];
                SPI_txBuffer[3] = MS5611_ADC_Buffer[2];
                HAL_SPI_Transmit(hspi, SPI_txBuffer, 4, SPI_SLAVE_TRANSACTION_TIMEOUT_MS);
            }
        }
	}
}


