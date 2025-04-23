/*
 * Header file for communicating with SX1280 Radio Module.
 * Datasheet: https://www.semtech.com/products/wireless-rf/24-ghz-transceivers/sx1280
 *
 *
 * Each command sent over SPI is a 1 byte OPCODE followed by an N byte value, which is configurable based on command.
 *
 */


#include <stdint.h>

#ifndef SX1280_H_
#define SX1280_H_

#define RADIO_PKT_LEN 22

#define START_LOG 1
#define STOP_LOG 2
#define CLEAR_FILE_SYS 3

/** BEGIN OPCODE DEFINITIONS (argument, return descriptions) **/
#define GET_STATUS 0xC0 // R: Status
#define WRITE_REGISTER 0x18 // address[15:8], address[7:0], data[0:n]
#define READ_REGISTER 0x19 // address[15:8], address[7:0], R: data[0:n-1]
#define WRITE_BUFFER 0x1A // offset, data[0:n]
#define READ_BUFFER 0x1B // offset, R: data[0:n-1]
#define SET_SLEEP 0x84	// sleepConfig
#define SET_STANDBY 0x80 // standbyConfig
#define SET_FS 0xC1
#define SET_TX 0x83 // periodBase, periodbaseCount[15:8], preiodBaseCount[7:0]
#define SET_RX 0x82 // periodBase, periodbaseCount[15:8], preiodBaseCount[7:0]
#define SET_RX_DUTY_CYCLE 0x94 // rxPeriodBase, rxPeriodBaseCount[15:8], rxPeriodBaseCount[7:0], sleep
#define SET_CAD 0xC5
#define SET_TX_CONTINUOUS_WAVE 0xD1
#define SET_TX_CONTINOUS_PREAMBLE0 0xD2
#define SET_PACKET_TYPE 0x8A // packetType
#define GET_PACKET_TYPE 0x03
#define SET_RF_FREQUENCY 0x86 // rfFrequency[23:16], rfFrequency[15:8], rfFrequency[7:0]
#define SET_TX_PARAMS 0x8E // power, rampTime
#define SET_CAD_PARAMS 0x88 // cadSymbolNum
#define SET_BUFFER_BASE_ADDRESS 0x8F // txBaseAddress, rxBaseAddress
#define SET_MODULATION_PARAMS 0x8B // modParam1, modParam2, modParam3
#define SET_PACKET_PARAMS 0x8C // packetParam1-7
#define GET_RX_BUFFER_STATUS 0x17 // R: payloadLength, rxBufferOffset
#define GET_PACKET_STATUS 0x1D // packetStatus (32Bit)
#define GET_RSSI_INST 0x1F
#define SET_DIO_IRQ_PARAMS 0x8D // IRQ_MASK, DIO_MASK
#define GET_IRQ_STATUS 0x15 // R: irq_status
#define CLR_IRQ_STATUS 0x97 // irqMask
#define SET_REGULATOR_MODE 0x96 // regulatorMode
#define SET_SAVE_CONTEXT 0xD5
#define SET_AUTO_FS 0x9E // 0:1
#define SET_AUTO_TX 0x98 // time
#define SET_PERF_COUNTER_MODE 0x9C // perfCounterMode
#define SET_LONG_PREAMBLE 0x9B // enable
#define SET_UART_SPEED 0x9D // uartSpeed
#define SET_RANGING_ROLES 0xA3 // slave/master
#define SET_ADVANCED_RANGING 0x9A// 0:1

/** END OPCODE DEFINITIONS **/


/** BEGIN ARGUMENT DEFINITIONS **/
// Function == SetPacketParameters
#define PREAMBLE_LENGTH_04_BITS 0x00
#define PREAMBLE_LENGTH_08_BITS 0x10
#define PREAMBLE_LENGTH_12_BITS 0x20
#define PREAMBLE_LENGTH_16_BITS 0x30
#define PREAMBLE_LENGTH_20_BITS 0x40
#define PREAMBLE_LENGTH_24_BITS 0x50
#define PREAMBLE_LENGTH_28_BITS 0x60
#define PREAMBLE_LENGTH_32_BITS 0x70
#define SYNC_WORD_LEN_1_B 0x00
#define SYNC_WORD_LEN_2_B 0x02
#define SYNC_WORD_LEN_3_B 0x04
#define SYNC_WORD_LEN_4_B 0x06
#define SYNC_WORD_LEN_5_B 0x08
#define RADIO_SELECT_SYNCWORD_OFF 0x00
#define RADIO_SELECT_SYNCWORD_1 0x10
#define RADIO_SELECT_SYNCWROD_2 0x20
#define RADIO_SELECT_SYNCWORD_1_2 0x30
#define RADIO_SELECT_SYNCWORD_3 0x40
#define RADIO_SELECT_SYNCWORD_1_3 0x50
#define RADIO_SELECT_SYNCWORD_2_3 0x60
#define RADIO_SELECT_SYNCWORD_1_2_3 0x70
#define RADIO_PACKET_FIXED_LENGTH 0x00
#define RADIO_PACKET_VARIABLE_LENGTH 0x20
#define RADIO_CRC_OFF 0x00
#define RADIO_CRC_1_BYTES 0x10
#define RADIO_CRC_2_BYTES 0x20
#define WHITENING_ENABLE 0x00
#define WHITENING_DISABLE 0x08
#define LORA_IQ_STD 0x40
#define LORA_IQ_INVERTED 0x00
#define LORA_CRC_ENABLE 0x20
#define LORA_CRC_DISABLE 0x00
#define EXPLICIT_HEADER 0x00
#define IMPLICIT_HEADER 0x80
#define PACKET_TYPE_GFSK 0x00
#define PACKET_TYPE_LORA 0x01
#define PACKET_TYPE_RANGING 0x02
#define PACKET_TYPE_FLRC 0x03
#define PACKET_TYPE_BLE 0x04
#define RADIO_SPREADING_FACTOR 0x70 //SF = 7
#define RADIO_BANDWIDTH 0x0A        //BW = 1600
#define RADIO_CODING_RATE 0x05      //LI 4/5 Coding Rate
#define MODULATION_PARAMS_REGISTER 0x0925
#define MODULATION_PARAMS_VALUE 0x32
#define FREQUENCY_ERROR_COMPENSATION_REGISTER 0x093C
#define FREQUENCY_ERROR_COMPENSATION_VALUE 0x1
#define LORA_PAYLOAD_LENGTH 0xff
#define MAX_POWER 31 //19 is 7dBm of output power ~33dBm after amps. Max is 0x31 which is 13dBm out
#define RAMP_TIME 0xE0
#define IRQ_MASK 0x03
#define DIO1_MASK  0x01
#define DIO2_MASK 0x02
#define DIO3_MASK 0x00
#define RF_FREQUENCY 0xB89D80
#define STDBY_MODE 0x00

void init_radio(void);
void init_radio_gfsk(void);
uint8_t radio_txPacket(uint8_t* packet, uint16_t packet_sz);
uint8_t radio_clearInterrupt(void);
void radio_RXMode(void);
int radio_getPktStatus(void);
uint8_t radio_getRXBufferStatus(uint8_t * packet);
uint8_t radio_rxPacket(uint8_t* packet, uint8_t offset, uint8_t length);
uint8_t radio_readRSSI(void);
uint8_t radio_readStatus(void);
uint8_t radio_checkMAC(void);
uint8_t radio_readInterrupt(void);
void radio_IDLEMode(void);
uint8_t radio_initAntennaDiv(void);
uint8_t radio_checkRxPkt(void);
uint8_t radio_set_packet_length(uint8_t);
uint8_t set_packet_length(uint8_t);
void set_packet_parameters_gfsk(uint8_t preamble_length, uint8_t sync_wrd_nbytes, uint8_t sync_wrd_match, uint8_t header_type, uint8_t payload_length, uint8_t crc_length, uint8_t white);
void set_packet_parameters_lora(uint8_t preamble_length, uint8_t header_type, uint8_t payload_length, uint8_t crc, uint8_t chip_invert);
uint8_t radio_telem_tx(uint8_t* buffer, uint16_t buffer_sz);
uint8_t radio_callsign_tx(void);
void radio_getStatus(void);
uint8_t radio_getIRQStatus(uint8_t status);
uint8_t radio_getCommandStatus(void);
uint8_t radio_getCircuitMode(void);
void radio_setTX(uint8_t periodBase, uint16_t periodBaseCount);
void radio_setRX(uint8_t periodBase, uint16_t periodBaseCount);
void radio_writeBuffer(uint8_t offset, uint8_t* data, uint8_t length);
void radio_readBuffer(uint8_t offset, uint8_t length, uint8_t* data);
void radio_setModulationParams(uint8_t mod1, uint8_t mod2, uint8_t mod3);
void radio_setPacketType(uint8_t packet_type);
void radio_setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);
void radio_writeRegister(uint16_t address, uint8_t * data, uint8_t length);
void radio_readRegister(uint16_t address, uint8_t length, uint8_t * data);
void radio_setTxParams(uint8_t power, uint8_t rampTime);
void radio_setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
uint8_t radio_getPacketType();
uint8_t radio_fetchPacket(uint8_t* packet);
void radio_setStandby(uint8_t stdbyMode);
void radio_setRfFrequency(uint32_t rfFreq);
void radio_setLongPreamble();
void radio_init();
uint8_t max_packet_length;

void radio_SPITransmitReceive(uint8_t* TX, uint8_t* RX, uint8_t length);

#endif /* RADIO_H_ */
