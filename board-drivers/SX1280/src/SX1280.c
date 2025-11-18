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
#include <stdlib.h>




//**************************************************
// * PRIVATE DEFINES
//**************************************************
#define SX1280_NOP 0x00
#define XTAL_FREQ 52000000UL // 52 MHz crystal frequency for SX1280

// 255 (max payload) + 5 (max command overhead) = 260
#define SX1280_MAX_SPI_BUFFER 260

//standby modes for SET_STANDBY command
#define STDBY_RC 0x00
#define STDBY_XOSC 0x01

// buffer offset for write operations
#define TX_BUFFER_OFFSET 0x00

// register definies
// datasheet section 4.2.1, page 26
#define SX1280_REG_SENSITIVITY_BOOST 0x0891
// from datasheet section 13.4.1, page 112
#define SX1280_REG_LORA_SF_CONFIG 0x0925
// datasheet section 14.4.1, page 131 (V3.3) - Frequency Error Compensation
#define SX1280_REG_FREQ_ERROR_COMP 0x093C

// IRQ Mask Definitions (from Datasheet Table 11-73)
#define SX1280_IRQ_TX_DONE              (1 << 0)
#define SX1280_IRQ_RX_DONE              (1 << 1)
#define SX1280_IRQ_SYNC_WORD_VALID      (1 << 2)
#define SX1280_IRQ_SYNC_WORD_ERROR      (1 << 3)
#define SX1280_IRQ_HEADER_VALID         (1 << 4)
#define SX1280_IRQ_HEADER_ERROR         (1 << 5)
#define SX1280_IRQ_CRC_ERROR            (1 << 6)
#define SX1280_IRQ_RX_TX_TIMEOUT        (1 << 14)
#define TX_BUFFER_OFFSET 0x00

//***********************************************************
// * PRIVATE VARIABLES
//***********************************************************
// static pointer to hardware config struct
static SX1280_Hal_t* sx1280_hal_config = NULL;


//***********************************************************
// * PRIVATE FUNCTION PROTOTYPES
//***********************************************************
static SX1280_Status_t SX1280_SPI_TransmitReceive(uint8_t* pTxData, uint8_t* pRxData, uint16_t size);
static SX1280_Status_t SX1280_WaitForReady(uint32_t timeout);


// public API function definitions ------------------//


SX1280_Status_t SX1280_Init(SX1280_Hal_t* hal_config) {
   //store hardware config
   sx1280_hal_config = hal_config;

   // perform hardware reset
   HAL_GPIO_WritePin(sx1280_hal_config->resetPort, sx1280_hal_config->resetPin, GPIO_PIN_RESET);
   vTaskDelay(20); //reset pulse
   HAL_GPIO_WritePin(sx1280_hal_config->resetPort, sx1280_hal_config->resetPin, GPIO_PIN_SET);
   vTaskDelay(50); //wait for chip to boot


   //set radio to state STDBY_RC
   uint8_t standby_mode = STDBY_RC;
   if(SX1280_SendCommand(SX1280_CMD_SET_STANDBY, &standby_mode, 1) != SX1280_OK) {
       return SX1280_ERROR;
   }

   // Set default LoRa packet type
   if (SX1280_SetPacketType(PACKET_TYPE_LORA) != SX1280_OK){
        return SX1280_ERROR;
   }

   // config default settings
    if (SX1280_SetRfFrequency(2400000000) != SX1280_OK) { // 2.4 GHz
        return SX1280_ERROR;
    }
    if (SX1280_SetBufferBaseAddress(0, 128) != SX1280_OK) {
        return SX1280_ERROR;
    }

    // UPDATED: Modulation Params for ~0.5 kbps (High Range / Low Data Rate)
    // SF12, BW 200kHz = ~0.476 kbps (Datasheet Table 3-5)
    // CR 4/8 = Maximum Error Correction (Overhead is high, but link budget is poor)
    if (SX1280_SetModulationParams(LORA_SF8, LORA_BW_400, LORA_CR_4_5) != SX1280_OK) {
        return SX1280_ERROR;
    }
    if (SX1280_SetPacketParams(12, LORA_EXPLICIT_HEADER, 255, LORA_CRC_ON, LORA_IQ_STANDARD) != SX1280_OK) {
        return SX1280_ERROR;
    }
    if (SX1280_SetTxParams(9, SX1280_RAMP_10_US) != SX1280_OK) { // DO NOT GO MORE THAN 10
        return SX1280_ERROR;
    }

    //important here, set high sensitivity mode
    if (SX1280_SetHighSensitivityMode() != SX1280_OK) {
        return SX1280_ERROR;
    }

    uint16_t irqMask = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE |
                       SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_TX_TIMEOUT |
                       SX1280_IRQ_HEADER_ERROR;


    uint16_t dio1Mask = irqMask;
    uint16_t dio2Mask = 0;
    uint16_t dio3Mask = 0;

    if (SX1280_SetDioIrqParams(irqMask, dio1Mask, dio2Mask, dio3Mask) != SX1280_OK) {
         return SX1280_ERROR;
    }


    return SX1280_OK;

}


SX1280_Status_t SX1280_WriteBuffer(uint8_t *data, uint8_t length)
{
    if (data == NULL || length == 0) {
        return SX1280_ERROR;
    }

    // safety check for the stack buffer size
    if ((2 + length) > SX1280_MAX_SPI_BUFFER) {
        return SX1280_ERROR;
    }

    // replacing VLA with a fixed stack buffer
    uint8_t cmd_buffer[SX1280_MAX_SPI_BUFFER];

    cmd_buffer[0] = SX1280_CMD_WRITE_BUFFER;
    cmd_buffer[1] = TX_BUFFER_OFFSET;

    memcpy(&cmd_buffer[2], data, length);

    // send cmd with data here
    return SX1280_SPI_TransmitReceive(cmd_buffer, NULL, 2 + length);
}

int16_t SX1280_ReadBuffer(uint8_t* data, uint8_t maxLength) {
    if (data == NULL || maxLength == 0) {
        return -1;
    }
    // get rx buffer status to find out where data is, how much
    uint8_t cmd[4] = {SX1280_CMD_GET_RX_BUFFER_STATUS, SX1280_NOP, SX1280_NOP, SX1280_NOP};
    uint8_t response[4];
    if(SX1280_SPI_TransmitReceive(cmd, response, 4) != SX1280_OK) {
        return -1;
    }


    uint8_t payloadLength = response[2];
    uint8_t rxBufferOffset = response[3];

    if (payloadLength == 0)
    {
        return 0; // this is no data
    }

    if (payloadLength > maxLength) {
        payloadLength = maxLength; // otherwise truncate to buffer size
    }

    // Safety check for stack buffer
    if ((3 + payloadLength) > SX1280_MAX_SPI_BUFFER) {
        return -1;
    }

    // REPLACED Malloc with fixed stack buffer
    uint8_t read_cmd[SX1280_MAX_SPI_BUFFER];
    uint8_t read_response[SX1280_MAX_SPI_BUFFER];

    read_cmd[0] = SX1280_CMD_READ_BUFFER;
    read_cmd[1] = rxBufferOffset;

    memset(&read_cmd[2], SX1280_NOP, 1 + payloadLength);

    SX1280_Status_t status = SX1280_SPI_TransmitReceive(read_cmd, read_response, 3 + payloadLength);

    // copy received data (skip 3 bytes, these are status bytes)
    if(status == SX1280_OK) {
        memcpy(data, &read_response[3], payloadLength);
    }

    return (status == SX1280_OK) ? payloadLength : -1;
}

SX1280_Status_t SX1280_SetPacketType(SX1280_PacketType_t packetType) {
    uint8_t packet_type = (uint8_t)packetType;
    return SX1280_SendCommand(SX1280_CMD_SET_PACKET_TYPE, &packet_type, 1);
}

SX1280_Status_t SX1280_SetRfFrequency(uint32_t frequency) {
    // calc frequency reg value
    // freq = (freq * 2^18) / XTAL_FREQ
    // Converting real frequency in Hz into a register value for SX1280's PLL
    // from datasheet, section 4.3, page 31: F_RF = F_xosc / 2^18 * rfFrequency
    // where F_RF is our desired, F_Xosc is crystal oscillator freq, rfFreq is 24-bit reg value, and
    // 2^18 is the PLL step resolution
    // left shift by 18 bits is the same as multiplying by 2^18
    uint32_t freq_reg;
    freq_reg = (uint32_t)(((uint64_t)frequency << 18) / XTAL_FREQ);

    uint8_t buf[3];
    buf[0] = (freq_reg >> 16) & 0xFF;
    buf[1] = (freq_reg >> 8) & 0xFF;
    buf[2] = freq_reg & 0xFF;

    return SX1280_SendCommand(SX1280_CMD_SET_RF_FREQUENCY, buf, 3);
}

SX1280_Status_t SX1280_SetModulationParams(SX1280_LoRa_SF_t sf, SX1280_LoRa_BW_t bw, SX1280_LoRa_CR_t cr) {
    // This function is specific to LoRa packet type based on parameters
    uint8_t buf[3];
    buf[0] = (uint8_t)sf;
    buf[1] = (uint8_t)bw;
    buf[2] = (uint8_t)cr;

    // Send the SetModulationParams command
    SX1280_Status_t status = SX1280_SendCommand(SX1280_CMD_SET_MODULATION_PARAMS, buf, 3);
    if (status != SX1280_OK) {
        return status;
    }

    // Add required register write based on SF (Datasheet Rev 3.3, Section 14.4.1, Step 5)
    uint8_t reg_val_sf;
    switch(sf) {
        case LORA_SF5:
        case LORA_SF6:
            reg_val_sf = 0x1E;
            break;
        case LORA_SF7:
        case LORA_SF8:
            reg_val_sf = 0x37;
            break;
        case LORA_SF9:
        case LORA_SF10:
        case LORA_SF11:
        case LORA_SF12:
            reg_val_sf = 0x32;
            break;
        default:
            reg_val_sf = 0x37; // Default defensively
            break;
    }
    // Write the required value to the SF config register 0x0925
    status = SX1280_WriteRegister(SX1280_REG_LORA_SF_CONFIG, &reg_val_sf, 1);
    if (status != SX1280_OK) {
        return status;
    }

    // Add required write to Frequency Error Compensation register 0x093C (Datasheet Rev 3.3, Section 14.4.1, Step 5)
    uint8_t freq_comp_val = 0x01;
    return SX1280_WriteRegister(SX1280_REG_FREQ_ERROR_COMP, &freq_comp_val, 1);
}

SX1280_Status_t SX1280_SetPacketParams(uint8_t preambleLengthVal,
                                        SX1280_LoRa_Header_Type_t headerType,
                                        uint8_t payloadLength,
                                        SX1280_LoRa_CRC_t crc,
                                        SX1280_LoRa_IQ_t invertIQ) {
    uint8_t buf[7];
    buf[0] = preambleLengthVal;     // packetParam1: PreambleLength (encoded per Table 14-50)
    buf[1] = (uint8_t)headerType;   // packetParam2: HeaderType
    buf[2] = payloadLength;         // packetParam3: PayloadLength
    buf[3] = (uint8_t)crc;          // packetParam4: CRC
    buf[4] = (uint8_t)invertIQ;     // packetParam5: InvertIQ
    buf[5] = 0x00;                  // packetParam6: Not used
    buf[6] = 0x00;                  // packetParam7: Not used

    return SX1280_SendCommand(SX1280_CMD_SET_PACKET_PARAMS, buf, 7);
}

SX1280_Status_t SX1280_SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
    uint8_t buf[2];
    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;

    return SX1280_SendCommand(SX1280_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2);
}

SX1280_Status_t SX1280_SetTxParams(int8_t power, uint8_t rampTime) {
    uint8_t buf[2];

    // -18 to +13, so constrain power to what we want
    if (power < -18) power = -18;
    if (power > 13) power = 13;

    // Power is offset by +18 to convert to register value (0-31)
    buf[0] = (uint8_t)(power + 18);
    buf[1] = rampTime;

    return SX1280_SendCommand(SX1280_CMD_SET_TX_PARAMS, buf, 2);
}

SX1280_Status_t SX1280_SetHighSensitivityMode(void) {
    uint8_t current_val;

    // read current value of register
    SX1280_Status_t status = SX1280_ReadRegister(SX1280_REG_SENSITIVITY_BOOST, &current_val, 1);
    if (status != SX1280_OK) {
        return status;
    }

    // modify value, set bits 7:6 to 0b11 (0x3)
    // or we do (current_val & 0x3F) | 0xC0
    uint8_t new_val = (current_val & 0x3F) | 0xC0;

    // write back modified value
    return SX1280_WriteRegister(SX1280_REG_SENSITIVITY_BOOST, &new_val, 1);
}


SX1280_Status_t SX1280_SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    // irqMask (which interrupts set flags in status register)
    buf[0] = (irqMask >> 8) & 0xFF;
    buf[1] = irqMask & 0xFF;

    // dio1Mask (which interrupts are routed to DIO1)
    buf[2] = (dio1Mask >> 8) & 0xFF;
    buf[3] = dio1Mask & 0xFF;

    // dio2Mask (which interrupts are routed to DIO2)
    buf[4] = (dio2Mask >> 8) & 0xFF;
    buf[5] = dio2Mask & 0xFF;

    // dio3Mask (which interrupts are routed to DIO3)
    buf[6] = (dio3Mask >> 8) & 0xFF;
    buf[7] = dio3Mask & 0xFF;

    return SX1280_SendCommand(SX1280_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}


SX1280_Status_t SX1280_SetTx(uint16_t timeout) {
    // Timeout in milliseconds
    // Uses periodBase = 0x02 (1ms steps per Table 11-24)
    // timeout = 0: no timeout, single mode (returns to STDBY_RC after packet sent)
    // timeout > 0: timeout active, returns to STDBY_RC on timeout or packet sent
    uint8_t buf[3];
    buf[0] = SX1280_PERIODBASE_1_MS;  // 1ms steps
    buf[1] = (timeout >> 8) & 0xFF;
    buf[2] = timeout & 0xFF;

    return SX1280_SendCommand(SX1280_CMD_SET_TX, buf, 3);
}

SX1280_Status_t SX1280_SetRx(uint16_t timeout) {
    // Timeout in milliseconds
    // Uses periodBase = 0x02 (1ms steps per Table 11-24)
    // timeout = 0x0000: no timeout, Rx single mode (returns to STDBY_RC after packet received)
    // timeout = 0xFFFF: continuous RX mode (stays in RX, can receive multiple packets)
    // timeout > 0: timeout active, returns to STDBY_RC on timeout or packet received
    uint8_t buf[3];
    buf[0] = SX1280_PERIODBASE_1_MS;  // 1ms steps
    buf[1] = (timeout >> 8) & 0xFF;
    buf[2] = timeout & 0xFF;

    return SX1280_SendCommand(SX1280_CMD_SET_RX, buf, 3);
}

/**
* @brief Sets the LoRa SyncWord.
* @note From datasheet 14.4.1 (page 133) and RadioLib source.
* The 8-bit word 0xXY is split.
* X (high nibble) is written to 0x944 [7:4]
* Y (low nibble) is written to 0x945 [7:4]
*/
SX1280_Status_t SX1280_SetLoRaSyncWord(uint8_t syncWord) {
    SX1280_Status_t status;
    uint8_t reg_val;

    // 1. Read/Modify/Write register 0x944 for high nibble
    status = SX1280_ReadRegister(0x944, &reg_val, 1);
    if (status != SX1280_OK) return status;

    reg_val = (reg_val & 0x0F) | (syncWord & 0xF0); // Set high nibble

    status = SX1280_WriteRegister(0x944, &reg_val, 1);
    if (status != SX1280_OK) return status;

    // 2. Read/Modify/Write register 0x945 for low nibble
    status = SX1280_ReadRegister(0x945, &reg_val, 1);
    if (status != SX1280_OK) return status;

    reg_val = (reg_val & 0x0F) | (syncWord << 4); // Set low nibble

    return SX1280_WriteRegister(0x945, &reg_val, 1);
}

SX1280_Status_t SX1280_SetStandby(uint8_t standbyConfig) {
    // standbyConfig should be STDBY_RC (0x00) or STDBY_XOSC (0x01)
    if (standbyConfig > 0x01) {
        return SX1280_ERROR;
    }
    return SX1280_SendCommand(SX1280_CMD_SET_STANDBY, &standbyConfig, 1);
}

//low level api func definitions
SX1280_Status_t SX1280_SendCommand(uint8_t opcode, uint8_t* buffer, uint16_t size) {
    if ((1 + size) > SX1280_MAX_SPI_BUFFER)
    {
        return SX1280_ERROR;
    }

    uint8_t cmd_buffer[SX1280_MAX_SPI_BUFFER];

    cmd_buffer[0] = opcode;
    if (buffer != NULL && size > 0)
    {
        memcpy(&cmd_buffer[1], buffer, size);
    }

    SX1280_Status_t status = SX1280_SPI_TransmitReceive(cmd_buffer, NULL, 1 + size);

    return status;
}

SX1280_Status_t SX1280_WriteRegister(uint16_t address, uint8_t* buffer, uint16_t size) {
    if ((3 + size) > SX1280_MAX_SPI_BUFFER)
    {
        return SX1280_ERROR;
    }

    uint8_t cmd_buffer[SX1280_MAX_SPI_BUFFER];

    cmd_buffer[0] = SX1280_CMD_WRITE_REGISTER;
    cmd_buffer[1] = (address >> 8) & 0xFF;
    cmd_buffer[2] = address & 0xFF;

    if (buffer != NULL && size > 0)
    {
        memcpy(&cmd_buffer[3], buffer, size);
    }

    SX1280_Status_t status = SX1280_SPI_TransmitReceive(cmd_buffer, NULL, 3 + size);

    return status;
}

SX1280_Status_t SX1280_ReadRegister(uint16_t address, uint8_t* buffer, uint16_t size) {
    if ((4 + size) > SX1280_MAX_SPI_BUFFER) {
        return SX1280_ERROR;
    }

    uint8_t cmd_buffer[SX1280_MAX_SPI_BUFFER];
    uint8_t rx_buffer[SX1280_MAX_SPI_BUFFER];

    cmd_buffer[0] = SX1280_CMD_READ_REGISTER;
    cmd_buffer[1] = (address >> 8) & 0xFF;
    cmd_buffer[2] = address & 0xFF;
    cmd_buffer[3] = SX1280_NOP;

    memset(&cmd_buffer[4], SX1280_NOP, size);

    SX1280_Status_t status = SX1280_SPI_TransmitReceive(cmd_buffer, rx_buffer, 4 + size);

    if (status == SX1280_OK && buffer != NULL && size > 0) {
        memcpy(buffer, &rx_buffer[4], size);
    }

    return status;
}

/**
* @brief Clears pending radio interrupts.
* @param irqMask 16-bit mask of interrupts to clear. 0xFFFF clears all.
*/
SX1280_Status_t SX1280_ClearIrqStatus(uint16_t irqMask) {
    uint8_t buf[2];
    buf[0] = (irqMask >> 8) & 0xFF; // MSB
    buf[1] = irqMask & 0xFF;        // LSB
    return SX1280_SendCommand(SX1280_CMD_CLEAR_IRQ_STATUS, buf, 2);
}

/**
* @brief Gets the 16-bit interrupt status register.
* @param irqStatus Pointer to a uint16_t to store the status.
*/
/**
* @brief Gets the 16-bit interrupt status register.
* @param irqStatus Pointer to a uint16_t to store the status.
*/
SX1280_Status_t SX1280_GetIrqStatus(uint16_t* irqStatus) {
    if (irqStatus == NULL) {
        return SX1280_ERROR;
    }

    uint8_t cmd[4];
    uint8_t response[4];

    cmd[0] = 0x15;
    cmd[1] = SX1280_NOP;
    cmd[2] = SX1280_NOP;
    cmd[3] = SX1280_NOP;

    SX1280_Status_t status = SX1280_SPI_TransmitReceive(cmd, response, 4);

    if (status != SX1280_OK) {
        return status;
    }

    *irqStatus = ((uint16_t)response[2] << 8) | response[3];
    return SX1280_OK;
}




   // private function definitions -------------------//


static SX1280_Status_t SX1280_SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
{
    HAL_StatusTypeDef hal_status;

    SX1280_Status_t wait_status = SX1280_WaitForReady(100);
    if (wait_status != SX1280_OK) {
        return wait_status;
    }

    // enter critical section for thread safety
    taskENTER_CRITICAL();


    // set NSS low to select the radio
    HAL_GPIO_WritePin(sx1280_hal_config->nssPort, sx1280_hal_config->nssPin, GPIO_PIN_RESET);


    // perform SPI transaction
    if (pRxData == NULL)
    {
        hal_status = HAL_SPI_Transmit(sx1280_hal_config->spiHandle, pTxData, size, SX1280_SPI_TIMEOUT_MS);
    }
    else
    {
        hal_status = HAL_SPI_TransmitReceive(sx1280_hal_config->spiHandle, pTxData, pRxData, size, SX1280_SPI_TIMEOUT_MS);
    }

    // set NSS high to deselect the radio
    HAL_GPIO_WritePin(sx1280_hal_config->nssPort, sx1280_hal_config->nssPin, GPIO_PIN_SET);
    // exit critical section
    taskEXIT_CRITICAL();

    if (hal_status == HAL_TIMEOUT) {
        return SX1280_TIMEOUT;
    } else if (hal_status != HAL_OK) {
        return SX1280_ERROR;
    }


    // wait for radio to be ready after transaction
    wait_status = SX1280_WaitForReady(100); // 100ms timeout after finishing
    if (wait_status != SX1280_OK) {
        return wait_status;
    }

    return SX1280_OK;
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
        //delay
        vTaskDelay(1);
    }
    return SX1280_OK;
}
