/*
 * MS5611_SIM.c
 *
 *  Created on: Oct 1, 2025
 *      Author: jackmh
 */

#include "MS5611_SIM.h"

static const uint16_t fixed_prom_values[8] = {
    0xABCD,
    40127,
    36924,
    23317,
    23282,
    33464,
    28312,
    0x1234
};

volatile static emulation_state_t state = WAITING_FOR_COMMAND;
static uint8_t prom_address_index = 0;
static uint8_t MS5611_ADC_Buffer[4] = {0};
static uint8_t UART_txBuffer[1] = {0};

void emulation_init() {
	// Disable SPI2 before reconfiguring
	SPI2->CR1 &= ~SPI_CR1_SPE;
	// IMPORTANT: Must enable hardware NSS in slave mode! Don't let the software option fool you
	SPI2->CR1 = 0
	    | (0 << SPI_CR1_BIDIMODE_Pos)   // 2-line full duplex
	    | (0 << SPI_CR1_BIDIOE_Pos)     // (ignored since BIDIMODE=0)
	    | (0 << SPI_CR1_CRCEN_Pos)      // No CRC
	    | (0 << SPI_CR1_CPOL_Pos)       // Clock polarity (0 = idle low, adjust if needed)
	    | (0 << SPI_CR1_CPHA_Pos)       // Clock phase (0 = sample on 1st edge, adjust if needed)
	    | (0 << SPI_CR1_MSTR_Pos)       // Slave mode
	    | (0 << SPI_CR1_LSBFIRST_Pos)   // MSB first
	    ;
	SPI2->CR2 = 0
	    | (7 << SPI_CR2_DS_Pos)         // Data size = 8-bit (DS=0b0111 = 8 bits)
	    | (1 << SPI_CR2_FRXTH_Pos)      // RXNE flag when FIFO has 8-bit (not 16-bit)
	    | (0 << SPI_CR2_NSSP_Pos)       // No NSS pulse mode (important!)
	    | (0 << SPI_CR2_TXDMAEN_Pos)    // No DMA
	    | (0 << SPI_CR2_RXDMAEN_Pos)    // No DMA
	    | (1 << SPI_CR2_RXNEIE_Pos)     // RXNE interrupt enable
	    | (1 << SPI_CR2_TXEIE_Pos)      // TXE interrupt enable
	    ;
	SPI2->CR1 |= SPI_CR1_SPE; // Enable SPI peripheral
	//SPI2->CR2 |= SPI_CR2_RXNEIE;// | SPI_CR2_FRXTH;// | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;// | SPI_CR2_TXEIE;
	//SPI2->CR1 |= SPI_CR1_SPE;  // Enable SPI peripheral
	*((__IO uint8_t*)&SPI2->DR) = 0x00; // preload with a dummy byte
	NVIC_EnableIRQ(SPI2_IRQn);
}

void emulation_IRQHandler(SPI_HandleTypeDef *hspi) {
    //static uint8_t dummy = 0x01;
	// RX: something arrived
    if (SPI_SR_RXNE & SPI2->SR) {
        uint8_t rx = *((__IO uint8_t*)&SPI2->DR); // clears RXNE
        emulation_state_machine_update(rx); // Update the state machine based on received byte
    }
    // TX: need a new byte for next cycle
    if (SPI_SR_TXE & SPI2->SR) {
        uint8_t tx = emulation_get_response(); // maybe dummy if no response queued
        *((__IO uint8_t*)&SPI2->DR) = tx; // clears TXE
    }
}

void emulation_state_machine_update(uint8_t rx_byte) {

	/*
	switch(state) {
		case WAITING_FOR_COMMAND:
			switch (rx_byte & 0xF0) {
	                default:
	                	prom_address_index = (rx_byte & 0x0E) >> 1;
	                	state = SENDING_PROM_DATA_LOW;
	                	break;
			}
			break;
		case SENDING_PROM_DATA_LOW:
			state = SENDING_PROM_DATA_HIGH;
			break;
		case SENDING_PROM_DATA_HIGH:
			state = WAITING_FOR_COMMAND; // Finished sending PROM data
			break;
		default:
			printf("idk what's going on either\n");
			break;
	}
*/

    switch (state) {
        case WAITING_FOR_COMMAND:
            // Handle command decoding
            switch (rx_byte & 0xF0) {
                case MS5611_CMD_PROM_READ_BASE:
                    prom_address_index = (rx_byte & 0x0E) >> 1; // Get PROM address index
                    state = SENDING_PROM_DATA_LOW; // Start sending PROM data
                    break;
                case MS5611_CMD_CONVERT_D1_BASE:
                    state = D1_CONVERSION_REQUESTED; // Handle D1 conversion request
                    break;
                case MS5611_CMD_CONVERT_D2_BASE:
                    state = D2_CONVERSION_REQUESTED; // Handle D2 conversion request
                    break;
                default:
                    switch (rx_byte) {
                        case MS5611_CMD_RESET:
                            state = RESETTING;
                            break;
                        case MS5611_CMD_ADC_READ:
                            state = SENDING_ADC_DATA_XL; // Start sending ADC data
                            break;
                        default:
                            // Unknown command, stay in WAITING_FOR_COMMAND
                            //printf("Unknown command: 0x%02X\n", rx_byte); // for debugging
                            break;
                    }
            }
            break;
        case SENDING_PROM_DATA_LOW:
            state = SENDING_PROM_DATA_HIGH;
            break;
        case SENDING_PROM_DATA_HIGH:
            state = WAITING_FOR_COMMAND; // Finished sending PROM data
            break;
        case RESETTING:
            state = WAITING_FOR_COMMAND; // Finished resetting
            break;
        case SENDING_ADC_DATA_XL:
            state = SENDING_ADC_DATA_L;
            break;
        case SENDING_ADC_DATA_L:
            state = SENDING_ADC_DATA_H;
            break;
        case SENDING_ADC_DATA_H:
            state = SENDING_ADC_DATA_XH;
            break;
        case SENDING_ADC_DATA_XH:
            state = WAITING_FOR_COMMAND; // Finished sending ADC data
            break;
        default:
        	break; // Should never happen
    }

}

uint8_t emulation_get_response() {
//    static uint8_t pretend_state = 0x01;
//    return pretend_state++;

	uint8_t response = 0x00; // Default dummy byte
    // Check the state machine for what we should respond with
    switch (state) {
        case SENDING_PROM_DATA_LOW: // these might be backwards
            response = (fixed_prom_values[prom_address_index] >> 8) & 0xFF; // MSB
            break;
        case SENDING_PROM_DATA_HIGH:
            response = (fixed_prom_values[prom_address_index] & 0xFF); // LSB
            break;
        case SENDING_ADC_DATA_XL:
            response = 0xFE; // First byte is always 0xFE for some reason
            break;
        case SENDING_ADC_DATA_L:
            response = MS5611_ADC_Buffer[0];
            break;
        case SENDING_ADC_DATA_H:
            response = MS5611_ADC_Buffer[1];
            break;
        case SENDING_ADC_DATA_XH:
            response = MS5611_ADC_Buffer[2];
            break;
        default:
            response = 0x00; // For all other states we don't want to send anything, so we make the response a dummy byte
            break;
    }
    return response;
}

void emulation_loop(UART_HandleTypeDef *huart) {
    if (state == (D1_CONVERSION_REQUESTED) || state == (D2_CONVERSION_REQUESTED)) {
        // Send the conversion commands over to UART and get a response to put in the ADC response buffer
        if (HAL_UART_Transmit(huart, UART_txBuffer, 1, UART_TRANSMIT_TIMEOUT_MS) != HAL_OK) {
            // TODO: error handling
        }
        if (HAL_UART_Receive(huart, MS5611_ADC_Buffer, 3, UART_RECEIVE_TIMEOUT_MS) != HAL_OK) {
            // TODO: error handling
        }
        state = WAITING_FOR_COMMAND; // Go back to waiting for command after handling conversion
    }
}
