/*
 * MS5611.h
 *
 *  Created on: January 12, 2025
 *      Author: jackmh
 */

#ifndef MS5611_H
#define MS5611_H

#include "main.h"
#include <math.h>

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

typedef enum {
    OSR_256 = 0,
    OSR_512 = 1,
    OSR_1024 = 2,
    OSR_2048 = 3,
    OSR_4096 = 4
} OSR;

typedef union {
    uint16_t bytes[6];
    struct {
        uint16_t C1;
        uint16_t C2;
        uint16_t C3;
        uint16_t C4;
        uint16_t C5;
        uint16_t C6;
    } constants;
} MS5611_PROM_t;

//Chip select
void MS5611_chipSelect(MS5611* BAR);

//Chip release
void MS5611_chipRelease(MS5611* BAR);

//Read register from barometer
HAL_StatusTypeDef MS5611_transmit(MS5611* BAR, uint8_t* tx_buffer, uint8_t num_bytes);

//Write register from barometer
HAL_StatusTypeDef MS5611_receive(MS5611* BAR, uint8_t* rx_buffer, uint8_t num_bytes);

// Abstraction for reading a single register from the chip
// Note: num_bytes is the combined size of tx_buffer and rx_buffer
int MS5611_read(MS5611* BAR, uint8_t* tx_buffer, uint8_t* rx_buffer, uint8_t num_bytes);

// Abstractions for writing a single register to the chip eg; for a command
int MS5611_write(MS5611* BAR, uint8_t reg);

// Software and memory reset
int MS5611_Reset(MS5611* BAR);

// Read the programmable read only memory
// Note: prom_buffer must be of size 6
int MS5611_readPROM(MS5611* BAR, MS5611_PROM_t* prom_buffer);

// Read the ADC result, assuming you have asked for a conversion
int MS5611_readADC(MS5611 *BAR, uint32_t *result);

// Pressure convert
// OSR is the "Over Sampling Rate" which determines the resolution of the pressure and temperature
int MS5611_presConvert(MS5611* BAR, uint32_t* pres_raw, OSR osr);

// Altitude convert
int MS5611_tempConvert(MS5611* BAR, uint32_t* temp_raw, OSR osr);

// Does some calculations using the constants from the PROM to give real pressure in mbar
int MS5611_compensateTemp(float* pres, uint32_t pres_raw, uint32_t temp_raw, MS5611_PROM_t* prom);

// Get pressure from barometer
int MS5611_getPres(MS5611* BAR, float* pres, MS5611_PROM_t* prom, OSR osr);

// Get rough altitude based on pressure
// You probably shouldn't use this for apogee detection
int MS5611_getAlt(MS5611* BAR, float* pres, float* alt, float BAR_SEA_LEVEL_PRESS);

#endif
