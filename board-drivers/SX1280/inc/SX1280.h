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

// SX1280 command opcodes, etc can be defined here or in the .c file

//from SX1280 datasheet table 12-1
#define SX1280_CMD_GET_STATUS               0xC0
#define SX1280_CMD_WRITE_REGISTER           0x18
#define SX1280_CMD_READ_REGISTER            0x19
#define SX1280_CMD_WRITE_BUFFER             0x1A
#define SX1280_CMD_READ_BUFFER              0x1B
#define SX1280_CMD_SET_SLEEP                0x84
#define SX1280_CMD_SET_STANDBY              0x80
#define SX1280_CMD_SET_FS                   0xC1
#define SX1280_CMD_SET_TX                   0x83
#define SX1280_CMD_SET_RX                   0x82
#define SX1280_CMD_SET_PACKET_TYPE          0x8A
#define SX1280_CMD_SET_RF_FREQUENCY         0x86
#define SX1280_CMD_SET_TX_PARAMS            0x8E
#define SX1280_CMD_SET_MODULATION_PARAMS    0x8B
#define SX1280_CMD_SET_PACKET_PARAMS        0x8C
#define SX1280_CMD_SET_BUFFER_BASE_ADDRESS  0x8F

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
  * 
  * additional info, func should perform basic hardware setup, reset it, check for comms and put into
  * known state (STDBY_RC)
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

SX1280_Status_t SX1280_WriteBuffer(uint8_t* data, uint8_t length);

/**
 * @brief reads received data payload from radio.
 * 
 * func checks for received packet, reads it form radio's
 * internal FIFO into buffer, returns number of bytes read
 * 
 * @param data pointer to buffer to store received data
 * @param maxLength maximum number of bytes to read into buffer
 * @return int16_t number of bytes read, or negative value on error
 */

int16_t SX1280_ReadBuffer(uint8_t* data, uint8_t maxLength);

// low level API prototypes

/**
 * @brief send a command to sx1280
 * 
 * @param opcode command opcode to send
 * @param buffer ptr to command parameters to send
 * @param size number of bytes in buffer
 * 
 */
void SX1280_SendCommand(uint8_t opcode, uint8_t* buffer, uint16_t size);

/**
 * @brief writes data to one or more registers
 * @param address starting register address to write to
 * @param buffer pointer to data to write
 * @param size number of bytes to write
 */
void SX1280_WriteRegister(uint16_t address, uint8_t* buffer, uint16_t size);

/**
 * @brief reads data from one or more registers
 * @param address starting register address to read from
 * @param buffer pointer to buffer to store read data
 * @param size number of bytes to read
 * 
 */
void SX1280_ReadRegister(uint16_t address, uint8_t* buffer, uint16_t size);




 

 // this marks the end of the header file, additional function prototypes can be added above this line
#endif // SX1280_H_