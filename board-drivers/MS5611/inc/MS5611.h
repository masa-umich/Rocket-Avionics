/*
 * MS5611.h
 *
 *  Created on: January 12, 2025
 *      Author: jackmh
 */

#ifndef MS5611_H
#define MS5611_H

#include "main.h"

#ifndef FreeRTOS_H
    #include "FreeRTOS.h"
    #include "task.h"
#else
    #error "This library is designed for use with FreeRTOS. Please include the FreeRTOS library in your project."
#endif

//Begin register map
#define MS5611_RESET            (uint8_t)0x1E
#define MS5611_D1_OSR_256       (uint8_t)0x40
#define MS5611_D1_OSR_512       (uint8_t)0x42
#define MS5611_D1_OSR_1024      (uint8_t)0x44
#define MS5611_D1_OSR_2048      (uint8_t)0x46
#define MS5611_D1_OSR_4096      (uint8_t)0x48
#define MS5611_D2_OSR_256       (uint8_t)0x50
#define MS5611_D2_OSR_512       (uint8_t)0x52
#define MS5611_D2_OSR_1024      (uint8_t)0x54
#define MS5611_D2_OSR_2048      (uint8_t)0x56
#define MS5611_D2_OSR_4096      (uint8_t)0x58
#define MS5611_ADC              (uint8_t)0x00
#define MS5611_PROM             (uint8_t)0xA0 //A0 to AE
//End register map

typedef struct {
    //SPI stuff
    SPI_HandleTypeDef* hspi;
    uint16_t SPI_TIMEOUT;
    GPIO_TypeDef * CS_GPIO_Port;
    uint16_t CS_GPIO_Pin;

    //Offset values
    float pres_offset;
    float alt_offset;
} MS5611;

enum OSR {
    OSR_256 = 0,
    OSR_512 = 1,
    OSR_1024 = 2,
    OSR_2048 = 3,
    OSR_4096 = 4
}

typedef union {
    uint16_t bytes[6];
    struct {
        uint16_t C1;
        uint16_t C2;
        uint16_t C3;
        uint16_t C4;
        uint16_t C5;
        uint16_t C6;
    }
} MS5611_PROM_t;

//Pressure convert
float MS5611_presConvert(MS5611 XL_Byte, uint8_t L_Byte, uint8_t H_Byte);

//Altitude convert
float MS5611_altConvert(MS5611 XL_Byte, uint8_t L_Byte, uint8_t H_Byte, float BAR_SEA_LEVEL_PRESS);

//Temperature convert
float MS5611_tempConvert(MS5611 L_Byte, uint8_t H_Byte);

//Chip select
void MS5611_chipSelect(MS5611* BAR);

//Chip release
void MS5611_chipRelease(MS5611* BAR);

//Read register from barometer
HAL_StatusTypeDef MS5611_read(MS5611* BAR, uint8_t reg_addr, uint8_t* rx_buffer, uint8_t num_bytes);

//Write register from barometer
HAL_StatusTypeDef MS5611_write(MS5611* BAR, uint8_t* tx_buffer, uint8_t num_bytes);

//Initialize barometer
int MS5611_init(MS5611* BAR);

//Get pressure from barometer
int MS5611_getPres(MS5611* BAR, float* pres);

//Get angular rate from barometer
int MS5611_getAlt(MS5611* BAR, float* alt, float BAR_SEA_LEVEL_PRESS);

//Get temperature from barometer
int MS5611_getTemp(MS5611* BAR, float* temp);

//Send command to barometer
int MS5611_send(MS5611* BAR, uint8_t cmd, uint8_t value);

//Return 0 is WHO_AM_I register can be read, 1 otherwise
int MS5611_whoami(MS5611* BAR);

//Waits / blocks for the pressure data to be ready
int MS5611_waitForPres(MS5611* BAR);

//Waits / blocks for the temperature data to be ready
int MS5611_waitForTemp(MS5611* BAR);

// Software and memory reset
int MS5611_Reset(MS5611* BAR);

#endif
