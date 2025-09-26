/**
 * @file SX1280.h
 * @brief Header file for the SX1280 radio transceiver driver.
 *
 * This file contains the function prototypes for the high-level API
 * to interact with the SX1280 chip
 *
 * MASA - University of Michigan
 */

#include <stdint.h>
#include "stm32h7xx_hal.h"

#ifndef SX1280_H_
#define SX1280_H_

/**
 * @brief status enum for driver functions
 */
typedef enum {
    SX1280_OK = 0,
    SX1280_ERROR,
    SX1280_BUSY,
    SX1280_TIMEOUT
} SX1280_Status_t;

/**
 * @brief structure to handle all necessary hardware handles/pins for the SX1280
 * instance of this struct should be passed to Initialize function
 */

 typedef struct {
    SPI_HandleTypeDef* spiHandle;
    GPIO_TypeDef* nssPort;
    uint16_t nssPin;
    GPIO_TypeDef* resetPort;
    uint16_t resetPin;
    GPIO_TypeDef* busyPort;
    uint16_t busyPin;
    GPIO_TypeDef* irq_port; // DIO1 pin
    uint16_t irq_pin;
 } SX1280_Hal_t;

 // PUBLIC API FUNCTION PROTOTYPES



 /**
  * @brief initializes the SX1280 radio module with the provided hardware configuration
  * @param hal_config Pointer to a structure containing the hardware configuration for the SX1280
  * @return status code indicating success or failure of the initialization process
  */

SX1280_Status_t SX1280_Init(const SX1280_Hal_t* hal_config);

/**
 * @brief transmit a data payload over the radio
 * 
 * this function writes a data buffer to radio's FIFO and initiates transmission
 * note this is a blocking call and will wait until transmission is complete
 * 
 * @param data pointer to the data buffer to be transmitted
 * @param length the number of bytes to transfer
 * @return SX1280_Status_t status of the transmission
 */

SX1280_Status_t SX1280_Transmit(const uint8_t* data, uint8_t length);

/**
 * @brief receive a data payload from the radio
 * this function waits for a packet to be received and reads it from the radio's FIFO
 * note this is a blocking call and will wait until a packet is received or timeout occurs
 * 
 * @param data pointer to the buffer where received data will be stored
 * @param max_length the maximum number of bytes to read into the buffer
 * @return number of bytes actually received, or negative value on error
 */

int SX1280_Receive(uint8_t* data, uint8_t max_length);


 

 // this marks the end of the header file, additional function prototypes can be added above this line
#endif // SX1280_H_