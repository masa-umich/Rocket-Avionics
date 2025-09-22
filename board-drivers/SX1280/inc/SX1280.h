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

#ifndef SX1280_H_
#define SX1280_H_

/**
 * \brief initializes the sx1280 radio transceiver module
 * 
 *  This function performs the necessary initialization steps to prepare the SX1280 module for operation.
 * It configures the SPI interface, sets up GPIO pins, and initializes the module with
 * default settings.
 * Prepares the chip for transmit and receive operations.
 * 
 * @return status code indicating success or failure of the initialization process.
 */

 int SX1280_Init(void);

 /**
  * @brief transmits a data buffer over the SX1280 radio module
  * 
  * @param txBuffer Pointer to the data buffer to be transmitted
  * @param size Size of the data buffer in bytes
  * @return status code indicating success or failure of the transmission process
  * 
  */
 int SX1280_Transmit(uint8_t* txBuffer, uint8_t size);

 /**
  * @brief reads a received data buffer from the SX1280 radio module
  * 
  * @param rxBuffer pointer to the buffer where the received data will be stored
  * @param size pointer to a variable where the size of the received data will be stored
  * @return status code indicating success or failure of the reception process
  */
 int SX1280_Receive(uint8_t* rxBuffer, uint8_t* size);


 

 // this marks the end of the header file, additional function prototypes can be added above this line
#endif /* RADIO_H_ */