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
#define SX1280_CMD_GET_STATUS 0xC0
#define SX1280_CMD_WRITE_REGISTER 0x18
#define SX1280_CMD_READ_REGISTER 0x19
#define SX1280_CMD_WRITE_BUFFER 0x1A
#define SX1280_CMD_READ_BUFFER 0x1B
#define SX1280_CMD_SET_SLEEP 0x84
#define SX1280_CMD_SET_STANDBY 0x80
#define SX1280_CMD_SET_FS 0xC1
#define SX1280_CMD_SET_TX 0x83
#define SX1280_CMD_SET_RX 0x82
#define SX1280_CMD_SET_PACKET_TYPE 0x8A
#define SX1280_CMD_SET_RF_FREQUENCY 0x86
#define SX1280_CMD_SET_TX_PARAMS 0x8E
#define SX1280_CMD_SET_MODULATION_PARAMS 0x8B
#define SX1280_CMD_SET_PACKET_PARAMS 0x8C
#define SX1280_CMD_SET_BUFFER_BASE_ADDRESS 0x8F
#define SX1280_CMD_GET_RX_BUFFER_STATUS 0x17


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


/**
 * @brief packet types supported by sx1280 (Table 11-42)
 *
 */


 typedef enum {
   PACKET_TYPE_GFSK = 0x00,
   PACKET_TYPE_LORA = 0x01,
   PACKET_TYPE_RANGING = 0x02,
   PACKET_TYPE_FLRC = 0x03,
   PACKET_TYPE_BLE = 0x04
 } SX1280_PacketType_t;




 /**
  * @brief LoRa spreading factor (table 14-47)
  */
 typedef enum {
   LORA_SF5 = 0x50,
   LORA_SF6 = 0x60,
   LORA_SF7 = 0x70,
   LORA_SF8 = 0x80,
   LORA_SF9 = 0x90,
   LORA_SF10 = 0xA0,
   LORA_SF11 = 0xB0,
   LORA_SF12 = 0xC0
 } SX1280_LoRa_SF_t;


 /**
  * @brief LoRa bandwidth settings (table 14-48)
  */
 typedef enum {
   LORA_BW_1600 = 0x0A,  // 1625 kHz
   LORA_BW_800 = 0x18, // 812.5 kHz
   LORA_BW_400 = 0x26, // 406.25 kHz
   LORA_BW_200 = 0x34, // 203.125
 } SX1280_LoRa_BW_t;


 /**
  * @brief LoRa coding rate settings (table 14-49)
  */


 typedef enum {
   LORA_CR_4_5 = 0x01,
   LORA_CR_4_6 = 0x02,
   LORA_CR_4_7 = 0x03,
   LORA_CR_4_8 = 0x04,
   LORA_CR_LI_4_5 = 0x05, // low data rate optimization enabled
   LORA_CR_LI_4_6 = 0x06,
   LORA_CR_LI_4_8 = 0x07
  } SX1280_LoRa_CR_t;


 /**
  * @brief LoRa packet header types (table 14-51)
  */
 typedef enum {
   LORA_EXPLICIT_HEADER = 0x00,
   LORA_IMPLICIT_HEADER = 0x80
 } SX1280_LoRa_Header_Type_t;


 /**
  * @brief LoRa crc settings (table 14-53)
  */
 typedef enum {
   LORA_CRC_OFF = 0x00,
   LORA_CRC_ON = 0x20
 } SX1280_LoRa_CRC_t;


 /**
  * @brief LoRa IQ setting (table 14-54)
  */
 typedef enum {
   LORA_IQ_STANDARD = 0x40,
   LORA_IQ_INVERTED = 0x00
 } SX1280_LoRa_IQ_t;


 // PUBLIC API FUNCTION PROTOTYPES ------------------//


/**
 * @brief initialize SX1280 radio transceiver
 *
 * Function performs basic hardware setup for the radio, resets it,
 * checks for comms, and puts into STDBY_RC mode
 * must be called before any other functions
 *
 * @param hal_config pointer to struct containing hardware config
 * (SPI Handle, GPIO ports, pins, etc)
 * @return SX1280_Status_t status of initialization
 */
SX1280_Status_t SX1280_Init(SX1280_Hal_t* hal_config);
SX1280_Status_t SX1280_WriteBuffer(uint8_t* data, uint8_t length);
int16_t SX1280_ReadBuffer(uint8_t* data, uint8_t maxLength);


// config prototypes
/**
* @brief set radio packet type (ex. LORA, GFSK).
* Really we are only using LORA(?)
*/
void SX1280_Status_t SX1280_SetPacketType(SX1280_PacketType_t packetType);


/**
* @brief set radio carrier freq
* @param frequency the frequency in Hz (ex. 2400000000 for 2.4 GHz)
*/
void SX1280_Status_t SX1280_SetRfFrequency(uint32_t frequency);


/**
* @brief set modulation param for current packet type
* @param sf spreading factor (LoRa only)
* @param bw bandwidth (LoRa only)
* @param cr coding rate (LoRa only)
*/
void SX1280_Status_t SX1280_SetModulationParams(SX1280_LoRa_SF_t sf, SX1280_LoRa_BW_t bw, SX1280_LoRa_CR_t cr);


/**
* @brief set packet params for current packet type
* @param preambleLength length of preamble in symbols
* @param headerType explicit or implicit header (LoRa only)
* @param payloadLength length of payload in bytes
* @param crcOn crc on or off (LoRa only)
* @param invertIQ standard or inverted IQ (LoRa only)
*/
void SX1280_Status_t SX1280_SetPacketParams(uint16_t preambleLength, SX1280_LoRa_Header_Type_t headerType, uint8_t payloadLength, SX1280_LoRa_CRC_t crc, SX1280_LoRa_IQ_t invertIQ);


/**
* @brief set base address for TX and RX buffers
* @param txBaseAddress start address for the TX buffer (0-255)
* @param rxBaseAddress start address for the RX buffer (0-255)
*/
void SX1280_Status_t SX1280_SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);


/**
* @brief sets transmit output power and ramp time
* @param power output power in dBm (-18 to +13 dBm)
* @param rampTime Ramp time constant.
*/
void SX1280_Status_t SX1280_SetTxParams(int8_t power, uint8_t rampTime);










// low level API prototypes --------------------//
/**
* @brief sends a command to sx1280
*
* @param opcode command opcode
* @param buffer pointer to command arguments
* @param size number of param bytes
*/
void SX1280_SendCommand(uint8_t opcode, uint8_t* buffer, uint16_t size);


void SX1280_WriteRegister(uint16_t address, uint8_t* buffer, uint16_t size);
void SX1280_ReadRegister(uint16_t address, uint8_t* buffer, uint16_t size);






// this marks the end of the header file, additional function prototypes can be added above this line
#endif // SX1280_H_

