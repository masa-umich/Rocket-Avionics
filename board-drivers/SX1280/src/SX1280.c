/*
 * Datasheet for SX1280 Radio:
 * https://media.digikey.com/pdf/Data%20Sheets/Semtech%20PDFs/SX1280-81_Rev3.2_Mar2020.pdf
 */

#include "SX1280.h"
#include "main.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "globals.h"


#define STATUS_HEADER_SIZE 1 // (in bytes)

extern SPI_HandleTypeDef hspi2;

extern uint8_t transmission_done;

//Changed to default
uint8_t TX_base = 0x00;
uint8_t RX_base = 0x00;
uint8_t max_packet_length = 252; // For TX, actual data 1 less due to status byte

uint8_t callsign[6];

enum ModulationMode{GFSK = 0, LORA = 1};

extern uint8_t radio_mode;

void radio_read_back_init() {
	uint8_t TXData[30] = {0};
	uint8_t RXData[30] = {0};

	TXData[0] = 0xC0;

	while(HAL_GPIO_ReadPin(BUSY_Radio_GPIO_Port, BUSY_Radio_Pin) == GPIO_PIN_SET) {};

	HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 1, 0xff);
	HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);

	uint8_t circuit = (RXData[0] & 0b11100000) >> 5;
	//uint8_t command = (RXData[0] & 0b00011100) >> 2;

	printf("Radio Circuit Status: \n");

	switch (circuit) {
	case 0x2:
		printf("STDBY_RC \n");
		break;
	case 0x3:
		printf("STDBY_XOSC \n");
		break;
	case 0x4:
		printf("FS \n");
		break;
	case 0x5:
		printf("RX \n");
	case 0x6:
		printf("TX \n");
		break;
	}

	printf("Radio Command Status: ");
	switch (circuit) {
	case 0x1:
		printf("Command Successfully Processed \n");
		break;
	case 0x2:
		printf("Data is available to the host \n");
		break;
	case 0x3:
		printf("Command Timed Out \n");
		break;
	case 0x4:
		printf("Command processing error \n");
		break;
	case 0x5:
		printf("Failure to execute command \n");
		break;
	case 0x6:
		printf("Command TX done \n");
		break;
	}

	TXData[0] = 0x03;

	while(HAL_GPIO_ReadPin(BUSY_Radio_GPIO_Port, BUSY_Radio_Pin) == GPIO_PIN_SET) {};

	HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 3, 0xff);
	HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);

	uint8_t packet_type = RXData[2];

	printf("Radio Packet Type: ");

	switch(packet_type) {
	case 0x00:
		printf("GFSK \n");
		break;
	case 0x01:
		printf("LORA NORMAL \n");
		break;
	case 0x02:
		printf("LORA RANGING \n");
		break;
	case 0x03:
		printf("FLRC \n");
		break;
	case 0x04:
		printf("BLUETOOTH \n");
		break;
	}



}


void init_radio_gfsk(void) {
    HAL_GPIO_WritePin(nRST_Radio_GPIO_Port, nRST_Radio_Pin, 1);
    HAL_Delay(1000);

    uint8_t TXData[30] = {0};
    uint8_t RXData[30] = {0};

    // Set into standby
    TXData[0] = 0x80;
    TXData[1] = 0x00;
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
    HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 2, 0xff);
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
    HAL_Delay(10);

    //Set pkt type to GFSK
    TXData[0] = 0x8A;
    TXData[1] = 0x00;
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
    HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 2, 0xff);
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
    HAL_Delay(10);

    //Set RF frequency to 2.410GHz
    TXData[0] = 0x86;
    TXData[1] = 0xB9;
    TXData[2] = 0x62;
    TXData[3] = 0x76;
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
    HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 4, 0xff);
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
    HAL_Delay(10);

    //Set TX/RX buffer base addresses
    TXData[0] = 0x8F;
    TXData[1] = TX_base;
    TXData[2] = RX_base;
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
    HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 3, 0xff);
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
    HAL_Delay(10);

    //Set modulation params
    TXData[0] = 0x8B;
    TXData[1] = 0xEF; // Bitrate = 125Kb/s, BW = 0.3MHz, Chosen b/c gives highest sensitivity
    TXData[2] = 0x07; // Modulation Index = 2 (Testing)
    TXData[3] = 0x10; // Bandwidth-Tim bit period product = 1
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
    HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 4, 0xff);
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
    HAL_Delay(10);

    //Set packet params
    TXData[0] = 0x8C;
    TXData[1] = 0x30; // Preamble length = 16 bits (At least 2 bytes needed)
    TXData[2] = 0x04; // 3 Byte Sync Word
    TXData[3] = 0x10; // Transmitter sends sync word 1, reciever expects only sync word 1
    TXData[4] = 0x20; // Variable Length Packet
    TXData[5] = 0x16; // Payload (22 bytes)
    /*** Manual States that reciever will filter-out all packets with size greater
     * than payload length, so this may need to change ***/
    TXData[6] = 0x20; // 2 Bytes of CRC
    TXData[7] = 0x08; // Whitening Disable

    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
    HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 8, 0xff);
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
    HAL_Delay(10);

    // Configuring Sync Word
    TXData[0] = 0x18;
    TXData[1] = 0x09; // 0x09D0 (Stored in 1, 2) is the address for bits (23:16) of sync word 1,
    TXData[2] = 0xD0; // Radio auto incrememnts to next byte address, for bits (15:8), (7:0)
    TXData[3] = 'T';
    TXData[4] = 'S';
    TXData[5] = 'M'; // This is just fun, sync word could be anything.

    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
    HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 6, 0xff);
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
    HAL_Delay(10);

    // Set TX power & ramp speed
    TXData[0] = 0x8E;
    TXData[1] = 31; // Max
    TXData[2] = 0xE0;
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
    HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 3, 0xff);
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
    HAL_Delay(10);

    //Set IRQs
    TXData[0] = 0x8D;
    TXData[1] = 0x00;
    TXData[2] = 0x03; // TXDone and RXDone Active
    TXData[3] = 0x00;
    TXData[4] = 0x01; // TXDone on DIO1
    TXData[5] = 0x00;
    TXData[6] = 0x02; // RXDone on DIO2
    TXData[7] = 0x00;
    TXData[8] = 0x00;
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
    HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 9, 0xff);
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
    HAL_Delay(10);

    // Read back some params to make sure comms are okay
    // Read back packet type, should be 0x00;
    TXData[0] = 0x03;
    TXData[1] = 0x00;
    TXData[2] = 0x00;
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
    HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 3, 0xff);
    HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
    HAL_Delay(10);

}

// Assumes packet size has already been set.
uint8_t radio_txPacket(uint8_t* packet, uint16_t packet_sz) {
	transmission_done = 0;

	// This command is used to write to the data buffer. The TX data buffer is then used in TX mode after
	// BUSY pin goes low.

	uint8_t TXData[packet_sz + 2];
	uint8_t RXData[packet_sz + 2];
	TXData[0] = WRITE_BUFFER;
	TXData[1] = TX_base;

	for (int i = 2; i < packet_sz+2; i++) {
		TXData[i] = packet[i-2];
	}

	radio_SPITransmitReceive(TXData, RXData, packet_sz+2);

	radio_setTX(0,0);

	while(!transmission_done){};

	radio_clearInterrupt();
	radio_setRX(0,0);

	return 0;
}

uint8_t radio_clearInterrupt(void){
	uint8_t TXData[3] = {0};
	uint8_t RXData[3] = {0};

	// Clear all active interrupts.
	TXData[0] = CLR_IRQ_STATUS;
	TXData[1] = 0xFF;
	TXData[2] = 0xFF;

	radio_SPITransmitReceive(TXData, RXData, 3);

	return(1);
}

void radio_RXMode(void){
	radio_setRX(0,0);
}

int radio_getPktStatus(void){
	uint8_t TXData[30] = {0};
	uint8_t RXData[30] = {0};
	TXData[0] = GET_PACKET_STATUS;
	TXData[1] = 0xff;
	TXData[2] = 0xff;
	TXData[3] = 0xff;
	TXData[4] = 0xff;
	TXData[5] = 0xff;
	TXData[6] = 0xff;

	radio_SPITransmitReceive(TXData, RXData, 7);

//	float RSSI = -1*((int8_t)RXData[2]/2.0); //Not sure if this calc is correct? RX_Buff needs cast to int8_t?
	snr = RXData[3];
	if(RXData[2] == 0){
		return 0; // Packet has an error
	}
	return 1;
}

/* Previously rxGetBufferStatus --> Returns packet length, as well as populates argument (*packet)
 * with data received from radio.
*/
uint8_t radio_fetchPacket(uint8_t* packet) {
	radio_getPktStatus();
	// Expecting to return rxPayloadLen on Byte 2, and rxSTartBufferPoint on Byte 3
	uint8_t TXData[30] = {0};
	uint8_t RXData[30] = {0};
	uint8_t offset = RX_base;
	uint8_t payload_len = 0x00;
	TXData[0] = GET_RX_BUFFER_STATUS;
	TXData[1] = 0x00;
	TXData[2] = 0x00;
	TXData[3] = 0x00;

	radio_SPITransmitReceive(TXData, RXData, 4);
	offset = RXData[3];
	payload_len = RXData[2];
	if(payload_len <=max_packet_length){
		radio_rxPacket(packet, offset, payload_len);
	}
	else{
		return 0;
	}
	return payload_len;
}


uint8_t radio_rxPacket(uint8_t* packet, uint8_t offset, uint8_t length){
	uint8_t TXData[256] = {0};

	TXData[0] = READ_BUFFER;
	TXData[1] = offset; //Change to radio Offset
	TXData[2] = length; //Read this length

	uint8_t temp_packet[length+3];

	radio_SPITransmitReceive(TXData, temp_packet, length+3);

	for (int i = 0; i < length; i++) {
		packet[i] = temp_packet[i + 3];
	}


	return 0;
}


/*
 * Sending telemetry relies heavily on the status header, also seen as the third argument of the
 * radio_txPacket(uint8_t *, uint8_t, uint8_t) function.
 *
 * As you can see, the status header is an 8 bit signifier, which allows the receiving end to properly reconstruct packets,
 * especially those that are larger than the max packet size, which currently for this radio is 0xCC, 204.
 *
 * The header can take on two forms, "callsign type," or "chunk number deliniation type".
 *
 * A packet is labeled as a callsign if the header is expressed as 0xFF. The receiver accepts this packet, and then throws it out.
 *
 * A packet labeled with chunk number delineation is expressed as 0b(4b-Numerator)(4b-Denominator). In this format, the four least
 * significant bits of the packet indicate the denominator, i.e. the total number of chunks the packet has been split up into. The
 * four most significant bits indicate the numerator, i.e. which chunk is currently being sent or received.
 *
 * For example, if the radio wishes to transmit a 300B packet, it will have to split this into 2 chunks. The first chunk will be sent
 * with header 0b00010010, i.e. (1/2). The second chunk will be sent with header (0b00100010), i.e. (2/2). They must be sent in increasing
 * numerator order, and the X LOWEST INDICES will be sent first, where X is the max packet size, so in this case, the first 204 data
 * values of the packet will be sent, and then data from indices 204-299 will be sent.
 *
 *	UwU
 */
uint8_t radio_telem_tx(uint8_t* buffer, uint16_t buffer_sz) {
	radio_txPacket(buffer, buffer_sz);
	return 0;
}

uint8_t radio_callsign_tx() {
	callsign[0] = 0x4B;
	callsign[1] = 0x45;
	callsign[2] = 0x38;
	callsign[3] = 0x50;
	callsign[4] = 0x43;
	callsign[5] = 0x56;

	//set_packet_length(7);

	//radio_writeBuffer(TX_base , callsign, 6);
	//radio_txPacket(callsign, 6, 0xff);
	//__disable_irq();
	// delay until write buffer is complete
	//while(HAL_GPIO_ReadPin(BUSY_Radio_GPIO_Port, BUSY_Radio_Pin) == GPIO_PIN_SET) {};
	// HAL_Delay(10);
	//radio_setTX(0,0);
	//__enable_irq();

	return 0;
}

uint8_t set_packet_length(uint8_t length) {
	if (length > max_packet_length) {
		return 1;
	}

	if (radio_mode == GFSK) {
		set_packet_parameters_gfsk(
				PREAMBLE_LENGTH_12_BITS,
				SYNC_WORD_LEN_1_B,
				RADIO_SELECT_SYNCWORD_1,
				RADIO_PACKET_VARIABLE_LENGTH,
				length, // payload length here
				RADIO_CRC_1_BYTES,
				WHITENING_DISABLE);
	} else if (radio_mode == LORA) {
		set_packet_parameters_lora(
				0b00100011, // 12 byte preamble
				EXPLICIT_HEADER,
				length, // payload length here
				LORA_CRC_ENABLE,
				LORA_IQ_STD);
	}
	return 0;
}

void set_packet_parameters_gfsk(uint8_t preamble_length, uint8_t sync_wrd_nbytes, uint8_t sync_wrd_match,
							uint8_t header_type, uint8_t payload_length, uint8_t crc_length, uint8_t white) {
	//Set packet params
	uint8_t TXData[8] = {0};
	uint8_t RXData[8] = {0};
	TXData[0] = SET_PACKET_PARAMS;

	// Preamble length -- defined by #defines in .h file.
	TXData[1] = preamble_length;

	// sync word length -- defined by #defines in .h file.
	TXData[2] = sync_wrd_nbytes;

	// radio sync word select -- defined by #defines in .h file.
	TXData[3] = sync_wrd_match;

	// header type -- defined by #defines in .h file.
	TXData[4] = header_type;

	// 0-255 in bytes
	TXData[5] = payload_length;

	// radio CRC selection -- defined by #defines in .h file.
	TXData[6] = crc_length;

	// whitening enable/disable -- defined by #defines in .h file.
	TXData[7] = white;

	HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi2, TXData, RXData, 8, 0xff);
	HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);

	while(HAL_GPIO_ReadPin(BUSY_Radio_GPIO_Port, BUSY_Radio_Pin) == GPIO_PIN_SET) {};
}

void set_packet_parameters_lora(uint8_t preamble_length, uint8_t header_type, uint8_t payload_length,
								uint8_t crc, uint8_t chip_invert) {
	uint8_t TXData[8] = {0};
	uint8_t RXData[8] = {0};
	TXData[0] = SET_PACKET_PARAMS;

	// Bits (7:4) = exponent.
	// Bits (3:0) = base
	// preamble_length = base*(2^(exponent))
	TXData[1] = preamble_length;

	// either EXPLICIT_HEADER or IMPLICIT_HEADER
	TXData[2] = header_type;

	// 0-255 (no more than 253 if CRC is enabled)
	TXData[3] = payload_length;

	// CRC_ENABLE OR CRC_DISABLE
	TXData[4] = crc;

	// LORA_IQ_STD (0x40) or LORA_IQ_INVERTED (0x00)
	TXData[5] = chip_invert;

	radio_SPITransmitReceive(TXData, RXData, 8);
}

// The host can retrieve the transceiver status directly through the GetStatus() command:
// this command can be issued at any time as long as it is not the very first command
// send over the interface
// GetStatus() was divided into getCommandStatus and getCircuitMode

// Command Status return values:
// 0x0: reserved
// 0x1: transceiver has successfully processed the command
// 0x2: data are available to host
// 0x3: command time-out
// 0x4: command processing error
// 0x5: failure to execute command
// 0x6: command Tx done

uint8_t radio_getCommandStatus(void){
	uint8_t TXData[30] = {0};
	uint8_t RXData[30] = {0};

	TXData[0] = GET_STATUS;

	radio_SPITransmitReceive(TXData, RXData, 1);

	uint8_t val = RXData[0];
	uint8_t cmd_status = val >> 2 & 0b111;

	return cmd_status;
}

// Circuit Mode return values:
// 0x0: reserved
// 0x1: reserved
// 0x2: STDBY_RC
// 0x3: STDBY_XOSC
// 0x4: FS
// 0x5: Rx
// 0x6: TX

uint8_t radio_getCircuitMode(void){
	uint8_t TXData[30] = {0};
	uint8_t RXData[30] = {0};

	TXData[0] = GET_STATUS;

	radio_SPITransmitReceive(TXData, RXData, 1);

	uint8_t val = RXData[0];
	uint8_t circuit_mode = val >> 5;

	return circuit_mode;
}


/*
 *	 SetTx() sets the device in transmit mode | Datasheet pg. 79
 *
 *	 periodBase       : step of RTC
 *	 periodBaseCount  : 16-bit parameter defining the number of steps used during time-out
 *
 */

void radio_setTX(uint8_t periodBase, uint16_t periodBaseCount){
	uint8_t TXData[4] = {0};
	uint8_t RXData[4] = {0};

	TXData[0] = SET_TX;
	TXData[1] = periodBase;
	TXData[2] = periodBaseCount >> 8;
	TXData[3] = periodBaseCount & 0xFF;

	radio_SPITransmitReceive(TXData, RXData, 4);

}

/*
 *   SetRx() sets the device in receive mode | Datasheet pg. 80
 *
 *	 periodBase       : step of RTC
 *	 periodBaseCount  : number of steps used during time-out
 *
 */

void radio_setRX(uint8_t periodBase, uint16_t periodBaseCount){
	uint8_t RXData[4] = {0};
	uint8_t TXData[4] = {0};

	TXData[0] = SET_RX;
	TXData[1] = periodBase;
	TXData[2] = periodBaseCount >> 8;
	TXData[3] = periodBaseCount & 0xFF;

	radio_SPITransmitReceive(TXData, RXData, 4);
}

/*
 *   WriteBuffer() writes the data payload to be transmitted
 *
 *   offset  : start of address
 *   data    : data payload
 *   length  : data payload length
 */
void radio_writeBuffer(uint8_t offset, uint8_t* data, uint8_t length){

	uint8_t TXData[256] = {0};
	uint8_t RXData[256] = {0};

	TXData[0] = WRITE_BUFFER;
	TXData[1] = offset;

	// start at 2 to continue writing data in TXData
	for(int i = 2; i < length + 2; ++i){
		// remove 2 so the index at data starts at offset + 0
		TXData[i] = data[i - 2];
	}

	radio_SPITransmitReceive(TXData, RXData, length+2);
}

/*
 *   ReadBuffer() allows reading (n-3) bytes of payload received
 *   starting at offset.
 *
 *   offset  : start of address
 *   length  : data payload length
 */
void radio_readBuffer(uint8_t offset, uint8_t length, uint8_t* data){
	uint8_t TXData[256] = {0};
	uint8_t RXData[256] = {0};

	TXData[0] = READ_BUFFER;
	TXData[1] = offset;
	// from 2 - 5 : NOP

	radio_SPITransmitReceive(TXData, RXData, length+2);

	for(int i = 0; i < length; ++i) {
		data[i] = RXData[i+3];
	}
}

/*
 * SetModulationParams() configures the modulation parameters of the radio
 *
 * mod1  : spreading factor
 * mod2  : bandwidth
 * mod3  : coding rate
 *
 */
void radio_setModulationParams(uint8_t mod1, uint8_t mod2, uint8_t mod3){

	uint8_t TXData[4] = {0};
	uint8_t RXData[4] = {0};

	TXData[0] = SET_MODULATION_PARAMS;
	TXData[1] = mod1;
	TXData[2] = mod2;
	TXData[3] = mod3;

	radio_SPITransmitReceive(TXData, RXData, 4);

	// Write 0x32 to reg 0x925 as per data sheet note on pg 131
	// Reason not specified, although has to do with SF selected.
	uint8_t modParam[1] = {FREQUENCY_ERROR_COMPENSATION_VALUE};
	radio_writeRegister(FREQUENCY_ERROR_COMPENSATION_REGISTER, modParam, 1);

	if(mod3 == 0x50 || mod3 == 0x60){
		modParam[0] = 0x1E;
	}
	if(mod3 == 0x70 || mod3 == 0x80){
		modParam[0] = 0x37;
	}
	if(mod3 == 0x90 || mod3 == 0xA0 || mod3 == 0xB0 || mod3 == 0xC0){
		modParam[0] = 0x32;
	}
	radio_writeRegister(MODULATION_PARAMS_REGISTER, modParam, 1);

	// Aditional register write required by data sheet pg 131

}

/*
 *  SetPacketType() sets the transceiver radio frame out of a choice of 5 different packet types.
 *  Datasheet pg. 85
 *
 */
void radio_setPacketType(uint8_t packet_type){

	uint8_t TXData[2] = {0};
	uint8_t RXData[2] = {0};


	TXData[0] = SET_PACKET_TYPE;
	TXData[1] = packet_type;

	radio_SPITransmitReceive(TXData, RXData, 2);

}

/*
 * setStandby() sets standby configuration to either STDBY_RC or STDBY_XOSC mode
 *
 * stdbyMode: either 0 for STDBY_RC mode or 1 for STDBY_XOSC mode
 */
void radio_setStandby(uint8_t stdbyMode){
	uint8_t TXData[2] = {0};
	uint8_t RXData[2] = {0};

	//sets first bit to opcode 0x80
	TXData[0] = SET_STANDBY;

	//sets second bit to standby configuration
	TXData[1] = stdbyMode;

	radio_SPITransmitReceive(TXData, RXData, 2);

}

/*
 *  SetBufferBaseAddress() d fixes the base address for the packet handing operation
 *  in Tx and Rx mode for all packet types.
 *  Datasheet pg. 89
 *
 */
void radio_setBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress){

	uint8_t TXData[3] = {0};
	uint8_t RXData[3] = {0};

	TXData[0] = SET_BUFFER_BASE_ADDRESS;
	TXData[1] = txBaseAddress;	//txBase
	TXData[2] = rxBaseAddress;	//rxBase

	radio_SPITransmitReceive(TXData, RXData, 3);
}
/*
 *  WriteRegister() writes a block of bytes in a data memory space starting at a specific address.
 *  Datasheet pg. 74
 *
 *  address  : register address
 *  data     : data to write on register
 *  length   : length of data array
 *
 */
void radio_writeRegister(uint16_t address, uint8_t * data, uint8_t length){


	uint8_t TXData[256] = {0};
	uint8_t RXData[256] = {0};

	TXData[0] = WRITE_REGISTER;
	TXData[1] = address >> 8;
	TXData[2] = address & 0xff;

	for(int i = 0; i < length; ++i){
		TXData[i + 3] = data[i];
	}

	radio_SPITransmitReceive(TXData, RXData, length + 3);
}
/*
 *  ReadRegister() reads a block of data starting at a given address.
 *  Datasheet pg. 75
 *
 *  address  : register address
 *  data     : data to write on register
 *  length   : length of data array
 *
 */
void radio_readRegister(uint16_t address, uint8_t length, uint8_t* data) {
	uint8_t TXData[256] = {0};
	uint8_t RXData[256] = {0};

	for(int i = 0; i < length + 1; ++i){
		TXData[i+3] = 0xff;
		RXData[i+4] = 10;
	}

	TXData[0] = READ_REGISTER;
	TXData[1] = address >> 8;
	TXData[2] = address & 0xff;

	radio_SPITransmitReceive(TXData, RXData, length + 4);

	for(int i = 0; i < length + 4; ++i){
		data[i] = RXData[i];
	}
}
/*
 * SetTXParams() sets the Tx output power using parameter power and the Tx ramp time using parameter rampTime. This
 * command is available for all packetType.
 * Datasheet pg. 87
 */
void radio_setTxParams(uint8_t power, uint8_t rampTime){

	uint8_t TXData[3] = {0};
	uint8_t RXData[3] = {0};

	TXData[0] = SET_TX_PARAMS;
	TXData[1] = power; //19 is 7dBm of output power ~33dBm after amps. Max is 31 which is 13dBm out
	TXData[2] = rampTime;

	radio_SPITransmitReceive(TXData, RXData, 3);
}
/*
 *  SetDioIrqParams() command is used to enable IRQs and to route IRQs to DIO pins.
 *
 *  Datasheet pg. 96
 */
void radio_setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask){

	uint8_t TXData[9] = {0};
	uint8_t RXData[9] = {0};

	TXData[0] = SET_DIO_IRQ_PARAMS;

	TXData[1] = (irqMask >> 8) & 0xff;
	TXData[2] = irqMask & 0xff;

	TXData[3] = (dio1Mask >> 8) & 0xff;
	TXData[4] = dio1Mask & 0xff;

	TXData[5] = (dio2Mask >> 8) & 0xff;
	TXData[6] = dio2Mask & 0xff;

	TXData[7] = (dio3Mask >> 8) & 0xff;
	TXData[8] = dio3Mask & 0xff;

	radio_SPITransmitReceive(TXData, RXData, 9);
}

/*
 *  GetPacketType() returns the current operating packet type of the radio
 *
 */
uint8_t radio_getPacketType(){

	uint8_t TXData[3] = {0};
	uint8_t RXData[3] = {0};

	TXData[0] = GET_PACKET_TYPE;
	TXData[1] = 0x00;
	TXData[2] = 0x00;

	radio_SPITransmitReceive(TXData, RXData, 3);
	return RXData[2];
}

/*
 * getIRQStatus() returns the 16 bit irq status
 *
 * status	:	input specifying which part of irqStatus is desired - 2 for irqStatus[15:8], 3 for irqStatus[7:0]
 */
uint8_t radio_getIRQStatus(uint8_t status){

	uint8_t TXData[4] = {0};
	uint8_t RXData[4] = {0};

	TXData[0] = GET_IRQ_STATUS;
	//1-3 NOP

	radio_SPITransmitReceive(TXData, RXData, 4);

	return RXData[status];
}
/*
 * clearIRQStatus() clears an irq flag in the irq register
 *
 * irqMask	:	mask that you clear the irq flag with
 */
void radio_clearIRQStatus(uint16_t irqMask){

	uint8_t TXData[3] = {0};
	uint8_t RXData[3] = {0};

	TXData[0] = CLR_IRQ_STATUS;
	TXData[1] = irqMask >> 0x08;
	TXData[2] = irqMask & 0xff;

	radio_SPITransmitReceive(TXData, RXData, 3);
}

/*
 * setSaveContext() is used to restore the configuration of the radio, stores the current configuration of the radio
 */
void radio_setSaveContext(){

	uint8_t TXData[1] = {0};
	uint8_t RXData[1] = {0};

	TXData[0] = SET_SAVE_CONTEXT;

	radio_SPITransmitReceive(TXData, RXData, 1);
}

/*
 * setAutoFS() is used to activate or deactivate the AutoFs feature
 *
 * toggle	:	int that is either 1 for enable or 0 for disable
 */
void radio_setAutoFS(uint8_t toggle){

	uint8_t TXData[2] = {0};
	uint8_t RXData[2] = {0};

	TXData[0] = SET_AUTO_FS;
	TXData[1] = toggle;

	radio_SPITransmitReceive(TXData, RXData, 2);
}

/*
 * setAutoTX() - allows transceiver to send a packet at a user programmable time
 *
 * time 	:	time to be set by user
 */
void radio_setAutoTX(uint16_t time){

	uint8_t TXData[3] = {0};
	uint8_t RXData[3] = {0};

	TXData[0] = SET_AUTO_TX;
	TXData[1] = time >> 0x08;
	TXData[2] = time & 0xff;

	radio_SPITransmitReceive(TXData, RXData, 3);
}

/*
 *	setCAD() -
 */
void radio_setCAD(){

	uint8_t TXData[1] = {0};
	uint8_t RXData[1] = {0};

	TXData[0] = SET_CAD;

	radio_SPITransmitReceive(TXData, RXData, 1);
}

/*
 *	setSleep() - used to set transceiver into sleep mode
 *
 *	sleepConfig		:	used to set specific parameters : if sleepConfig[1] == 1, data buffer will be in retention mode
 *														  sleepConfig[0]: 0 - DataRAM is flushed in sleep mode, 1 - DataRAM is in retention mode in sleep mode
 */
void radio_setSleep(uint8_t sleepConfig){

	uint8_t TXData[2] = {0};
	uint8_t RXData[2] = {0};

	TXData[0] = SET_SLEEP;
	TXData[1] = sleepConfig;

	radio_SPITransmitReceive(TXData, RXData, 2);
}


/*
 * setLongPreamble() is used to set the transceiver into long preamble mode
 *
 */
void radio_setLongPreamble(){

	uint8_t TXData[2] = {0};
	uint8_t RXData[2] = {0};

	TXData[0] = SET_LONG_PREAMBLE;
	TXData[1] = 0x01;

	radio_SPITransmitReceive(TXData, RXData, 2);
}



/*
 * setRfFrequency() - sets frequency of rfFrequency mode
 * rfFreq: 24 bit rfFrequency band
 */
void radio_setRfFrequency(uint32_t rfFreq){

	uint8_t TXData[4] = {0};
	uint8_t RXData[4] = {0};

	//sets first bit to 0x86
	TXData[0] = SET_RF_FREQUENCY;

	//sets second bit to rfFrequency[23:16]
	TXData[1] = (rfFreq >> 16) & 0xff;

	//sets third bit to rfFrequency[15:8]
	TXData[2] = (rfFreq >> 8) & 0xff;

	//sets final bit to rfFrequency[7:0]
	TXData[3] = rfFreq & 0xff;

	radio_SPITransmitReceive(TXData, RXData, 4);
}

int8_t foo_adder(int8_t a, int8_t b){
	return a + b;
}

/*
 * RADIO INITIALIZATION FUNCTION
 */
void radio_init(){
	HAL_GPIO_WritePin(nRST_Radio_GPIO_Port, nRST_Radio_Pin, 1);
	HAL_Delay(1000);

	//Set into standby
	radio_setStandby(STDBY_MODE);

	//Set pkt type to LORA
	radio_setPacketType(PACKET_TYPE_LORA);

	//Set RF frequency to ~2.4GHz
	radio_setRfFrequency(RF_FREQUENCY);

	//Set TX/RX buffer base addresses
	radio_setBufferBaseAddress(TX_base, RX_base);

	//Set modulation params
	radio_setModulationParams(RADIO_SPREADING_FACTOR, RADIO_BANDWIDTH, RADIO_CODING_RATE);

	//Set packet params
	set_packet_length(max_packet_length);

	//Set TX power & ramp speed
	radio_setTxParams(MAX_POWER, RAMP_TIME);

	//Set IRQs
	radio_setDioIrqParams(IRQ_MASK, DIO1_MASK, DIO2_MASK, DIO3_MASK);

	//Read back some params to make sure comms are okay
	//Read back packet type, should be 0x01;
	radio_getPacketType();

}

void radio_SPITransmitReceive(uint8_t* TX, uint8_t* RX, uint8_t length){
	//while(!transmission_done){} // Check to avoid breaking things if radio not done transmitting
	while(HAL_GPIO_ReadPin(BUSY_Radio_GPIO_Port, BUSY_Radio_Pin) == GPIO_PIN_SET) {};

	__disable_irq();
	HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi2, TX, RX, length, 0xff);
	HAL_GPIO_WritePin(CS_Radio_GPIO_Port, CS_Radio_Pin, 1);
	__enable_irq();

	while(HAL_GPIO_ReadPin(BUSY_Radio_GPIO_Port, BUSY_Radio_Pin) == GPIO_PIN_SET) {};

}
