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
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"


// for clarity, this section is to hold private defines -------------------//

// add SX1280 command opcodes, etc here


//

// private static variables --------------------------//
// static pointer to hardware config struct
static const SX1280_Hal_t* sx1280_hal_config;

// private function prototypes ----------------------//

/**
 * @brief private helper func for thread-safe spi comms
 * 
 * this function handles low-level detail of SPI transact, inclding
 * NSS chip select, busy pin wait
 * wrapped in FreeRTOS critical section for thread safety
 * 
 * @param pTxData pointer to data to be transmitted
 * @param pRxData pointer to buffer for received data
 * @param size number of bytes to transfer
 */
static void SX1280_SPI_TransmitReceive(uint8_t* pTxData, uint8_t* pRxData, uint16_t size);

/**
 * @brief waits for busy pin to go low, indicating chip is ready
 * @param timeout maximum time to wait in ms
 * @return SX1280_Status_t SX1280_OK if ready, SX1280_TIMEOUT if timeout
 */
static SX1280_Status_t SX1280_WaitForReady(uint32_t timeout);

// public API function definitions ------------------//

SX1280_Status_t SX1280_Init(const SX1280_Hal_t* hal_config) {
    // store hal config
    sx1280_hal_config = hal_config;

    //hardware reset sequence to be done here

    //set radio to known state here, ex standby mode

    //further configs ex. packet type, freq, etc, here

    //return success for now
    return SX1280_OK;
}

SX1280_Status_t SX1280_WriteBuffer(uint8_t* data, uint16_t length) {
    // implementation to be added
    //involves setting packet parameters writing payload to
    //radio buffer, issuing transmit comand
    (void)data;
    (void)length;

    return SX1280_OK;
}

int16_t SX1280_ReadBuffer(uint8_t* data, uint8_t maxLength) {
    // implmementation to be added
    // involves checking IRQ status, getting received packet's
    //length, reading payload from radio buffer
    (void)data;
    (void)maxLength;
    return 0; //return number of bytes read
}

// private function definitions -------------------//

static void SX1280_SPI_TransmitReceive(uint8_t* pTxData, uint8_t* pRxData, uint16_t size) {
    // wait for radio to not be busy before starting SPI transaction
    SX1280_WaitForReady(100); //100ms timeout

    //enter critical section for thread safety
    taskENTER_CRITICAL();

    //set NSS low to select the radio
    HAL_GPIO_WritePin(sx1280_hal_config->nssPort, sx1280_hal_config->nssPin, GPIO_PIN_RESET);

    //perform SPI transaction
    if (pRxData == NULL)
    {
        HAL_SPI_Transmit(sx1280_hal_config->spiHandle, pTxData, size, HAL_MAX_DELAY);
    }
    else
    {
        HAL_SPI_TransmitReceive(sx1280_hal_config->spiHandle, pTxData, pRxData, size, HAL_MAX_DELAY);
    }
    //set NSS high to deselect the radio
    HAL_GPIO_WritePin(sx1280_hal_config->nssPort, sx1280_hal_config->nssPin, GPIO_PIN_SET);
    //exit critical section
    taskEXIT_CRITICAL();

    //wait for radio to be ready after transaction
    SX1280_WaitForReady(100); //100ms timeout

}

static SX1280_Status_t SX1280_WaitForReady(uint32_t timeout) {
    uint32_t startTick = xTaskGetTickCount();
    while (HAL_GPIO_ReadPin(sx1280_hal_config->busyPort, sx1280_hal_config->busyPin) == GPIO_PIN_SET) {
        if ((xTaskGetTickCount() - startTick) > timeout) {
            return SX1280_TIMEOUT;
        }
    }
    return SX1280_OK;
}