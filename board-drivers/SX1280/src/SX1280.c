/**
 * @file SX1280.c
 * @brief Driver for the SX1280 radio transceiver
 *
 * This file implements the high-level API and low-level SPI communication
 * for the SX1280 chip, designed for an STM32 platform using HAL and FreeRTOS.
 *
 * MASA - University of Michigan
 */

#include "SX1280.h"
#include "stm32h7xx_hal.h" // adjust based on stm32 series
#include "FreeRTOS.h"
#include "task.h"

// private function prototype for SPI communication
static void SX1280_SPI_TransmitReceive(uint8_t *txData, uint8_t *rxData, uint16_t size);

/**
 * @brief private helper function to handle SPI transmit and receive operations
 *
 * this function wraps the STM32 HAL SPI communication calls in a FreeRTOS
 * critical section to ensure thread safety
 *
 * @param txData Pointer to the data to be transmitted
 * @param rxData Pointer to the buffer to store received data
 * @param size   Number of bytes to transmit/receive
 */
static void SX1280_SPI_TransmitReceive(uint8_t *txData, uint8_t *rxData, uint16_t size)
{
    // enter critical section to prevent task switching during SPI communication
    taskENTER_CRITICAL();

    // placeholder for HAL SPI Transmit/Receive call
    // ex: HAL_SPI_TransmitReceive(&hspi1, txData, rxData, size, HAL_MAX_DELAY);
    // TODO:add chip select (NSS) pin handling here (set low before, high after)

    // exit critical section
    taskEXIT_CRITICAL();
}

/**
 * @brief init the SX1280 radio transceiver.
 *
 * @return int Status code (0 for success).
 */
int SX1280_Init(void)
{
    //TODO: implement initialization sequence based on SX1280 datasheet
    // this will involve sending a series of commands via the SPI wrapper
    return 0;
}

/**
 * @brief transmits a data buffer over the SX1280 radio
 *
 * @param txBuffer pointer to the data buffer to be transmitted
 * @param size number of bytes to transmit
 * @return int status code (0 for success)
 */
int SX1280_WriteBuffer(uint8_t *txBuffer, uint8_t size)
{
    //TODO: implement the logic to write data to the radio's buffer
    // and initiate transmission
    return 0;
}

/**
 * @brief reads a received data buffer from the SX1280 radio
 *
 * @param rxBuffer Pointer to the buffer where the received data will be stored
 * @param size Pointer to a variable that will store the size of the received data
 * @return int Status code (0 for success)
 */
int SX1280_ReadBuffer(uint8_t *rxBuffer, uint8_t *size)
{
    //TODO: implement the logic to check for received data
    // read the payload and its length from the radio's buffer
    return 0;
}

// additional funcs here
