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
