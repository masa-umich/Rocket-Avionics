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
#include <string.h>




//**************************************************
// * PRIVATE DEFINES
//**************************************************
#define SX1280_NOP 0x00
#define XTAL_FREQ 52000000UL // 52 MHz crystal frequency for SX1280




//standby modes for SET_STANDBY command
#define STDBY_RC 0x00
#define STDBY_XOSC 0x01


//***********************************************************
// * PRIVATE VARIABLES
//***********************************************************
// static pointer to hardware config struct
static const SX1280_Hal_t* sx1280_hal_config = NULL;


//***********************************************************
// * PRIVATE FUNCTION PROTOTYPES
//***********************************************************
/**
* @brief private helper func for thread-safe SPI comms
*
* This func handles low-level details of SPI transactions, including
* Chip select, waiting for busy line
* Wrapped in FreeRTOS critical sections to ensure atomicity in multi-threaded
* environment, prevents context switches during SPI transactions
*
* @param pTxData pointer to data to transmit
* @param pRxData pointer to buffer for received data
* @param size number of bytes to transfer
*/
static void SX1280_SPI_TransmitReceive(uint8_t* pTxData, uint8_t* pRxData, uint16_t size);
static SX1280_Status_t SX1280_WaitForReady(uint32_t timeout);


// public API function definitions ------------------//


SX1280_Status_t SX1280_Init(SX1280_Hal_t* hal_config) {
   //store hardware config
   sx1280_hal_config = hal_config;


   // perform hardware reset
   HAL_GPIO_WritePin(sx1280_hal_config->resetPort, sx1280_hal_config->resetPin, GPIO_PIN_RESET);
   HAL_Delay(20); //reset pulse
   HAL_GPIO_WritePin(sx1280_hal_config->resetPort, sx1280_hal_config->resetPin, GPIO_PIN_SET);
   HAL_DELAY(50); //wait for chip to boot


   //set radio to state STDBY_RC
   uint8_t standby_mode = STDBY_RC;
   SX1280_SendCommand(SX1280_CMD_SET_STANDBY, &standby_mode, 1);


   // default to LoRa packet type
   SX1280_SetPacketType(PACKET_TYPE_LORA);
   SX1280_SetRfFrequency(2400000000); //2.4 GHz
   SX1280_SetBufferBaseAddress(0, 128);
   SX1280_SetModulationParams(LORA_SF8, LORA_BW_400, LORA_CR_4_5);  //NOTEHERE IS WHERE TO ACHIEVE EFFECTIVE BIT RATES. REF. TABLE
   SX1280_SetPacketParams(12, LORA_EXPLICIT_HEADER, 255, LORA_CRC_ON, LORA_IQ_STANDARD);
   SX1280_SetTxParams(0, 0x80); //0dbm, 10us ramp time
   return SX1280_OK;
}


SX1280_Status_t SX1280_WriteBuffer(uint8_t *data, uint8_t length)
{
   uint8_
}


   // private function definitions -------------------//


   static void SX1280_SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
   {
       // wait for radio to not be busy before starting SPI transaction
       SX1280_WaitForReady(100); // 100ms timeout


       // enter critical section for thread safety
       taskENTER_CRITICAL();


       // set NSS low to select the radio
       HAL_GPIO_WritePin(sx1280_hal_config->nssPort, sx1280_hal_config->nssPin, GPIO_PIN_RESET);


       // perform SPI transaction
       if (pRxData == NULL)
       {
           HAL_SPI_Transmit(sx1280_hal_config->spiHandle, pTxData, size, HAL_MAX_DELAY);
       }
       else
       {
           HAL_SPI_TransmitReceive(sx1280_hal_config->spiHandle, pTxData, pRxData, size, HAL_MAX_DELAY);
       }
       // set NSS high to deselect the radio
       HAL_GPIO_WritePin(sx1280_hal_config->nssPort, sx1280_hal_config->nssPin, GPIO_PIN_SET);
       // exit critical section
       taskEXIT_CRITICAL();


       // wait for radio to be ready after transaction
       SX1280_WaitForReady(100); // 100ms timeout
   }


   static SX1280_Status_t SX1280_WaitForReady(uint32_t timeout)
   {
       uint32_t startTick = xTaskGetTickCount();
       while (HAL_GPIO_ReadPin(sx1280_hal_config->busyPort, sx1280_hal_config->busyPin) == GPIO_PIN_SET)
       {
           if ((xTaskGetTickCount() - startTick) > timeout)
           {
               return SX1280_TIMEOUT;
           }
       }
       return SX1280_OK;
   }

