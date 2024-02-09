/*
 * LPS22HBTR.h
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#ifndef LPS22HBTR_H	// Begin header include protection
#define LPS22HBTR_H

#include "main.h"

#ifndef FreeRTOS_H
    #include "FreeRTOS.h"
    #include "task.h"
#else
    #error "This library is designed for use with FreeRTOS. Please include the FreeRTOS library in your project."
#endif

// Begin register map
#define BAR_WHO_AM_I_ADDR		(uint8_t)0x0F

// todo: add register map

// End register map

typedef struct {
    //SPI stuff
    SPI_HandleTypeDef* hspi;
    uint16_t SPI_TIMEOUT;
    GPIO_TypeDef * CS_GPIO_Port;
    uint16_t CS_GPIO_Pin;

    //Offset values
    float pres_offset;
    float alt_offset;
} BAR;

//Pressure convert
float BAR_presConvert(uint8_t H_Byte, uint8_t L_Byte);

//Altitude convert
float BAR_altConvert(uint8_t H_Byte, uint8_t L_Byte);

//Chip select
void BAR_chipSelect(BAR* BAR);

//Chip release
void BAR_chipRelease(BAR* BAR);

//Read register from barometer
HAL_StatusTypeDef BAR_read(BAR* BAR, uint8_t reg_addr, uint8_t* rx_buffer, uint8_t num_bytes);

//Write register from barometer
HAL_StatusTypeDef BAR_write(BAR* BAR, uint8_t* tx_buffer, uint8_t num_bytes);

//Initialize BAR
int BAR_init(BAR* BAR);

//Get pressure from BAR
int BAR_getPres(BAR* BAR, float* pres);

//Get angular rate from BAR
int BAR_getAlt(BAR* BAR, float* alt);

//Send command to barometer
int BAR_send(BAR* BAR, uint8_t cmd, uint8_t value);

#endif
