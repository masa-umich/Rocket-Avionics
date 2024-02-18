/*
 * LPS22HBTR.h
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#ifndef LPS22HBTR_H	// Begin header include protection
#define LPS22HBTR_H

#include "main.h"
#include <math.h>

#ifndef FreeRTOS_H
    #include "FreeRTOS.h"
    #include "task.h"
#else
    #error "This library is designed for use with FreeRTOS. Please include the FreeRTOS library in your project."
#endif

// Begin register map
#define BAR_WHO_AM_I_REG_ADDR		    (uint8_t)0x0F
#define BAR_CTRL1_C                     (uint8_t)0x12
#define BAR_CTRL2_C                     (uint8_t)0x13
#define BAR_CTRL3_C                     (uint8_t)0x14
#define BAR_PRESS_OUT_XL                (uint8_t)0x28
#define BAR_PRESS_OUT_L                 (uint8_t)0x29
#define BAR_PRESS_OUT_H                 (uint8_t)0x2A
#define BAR_TEMP_OUT_L                  (uint8_t)0x2B
#define BAR_TEMP_OUT_H					(uint8_t)0x2C
// todo: the rest of these

// End register map

// Constants
#define BAR_WHO_AM_I_REG_VAL            (uint8_t)0xB1
#define BAR_DEFAULT_CONF_CTRL1_C        (uint8_t)0xA0 //10100000 75hz with sequential register update
#define BAR_DEFAULT_CONF_CTRL2_C        (uint8_t)0x00 //00010000 enable sequential register update
#define BAR_DEFAULT_CONF_CTRL3_C        (uint8_t)0x00 //00000000
#define BAR_PRES_SCALING_FACTOR         (uint16_t)4096 //Given in datasheet as 4096 LSB/hPa
#define BAR_TEMP_SCALING_FACTOR         (uint16_t)100 //Given in datasheet as 100 LSB/degC (section 3.1)

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
float BAR_presConvert(uint8_t XL_Byte, uint8_t H_Byte, uint8_t L_Byte);

//Altitude convert
float BAR_altConvert(uint8_t XL_Byte, uint8_t H_Byte, uint8_t L_Byte, float BAR_SEA_LEVEL_PRESS);

//Temperature convert
float BAR_tempConvert(uint8_t L_Byte, uint8_t H_Byte);

//Chip select
void BAR_chipSelect(BAR* BAR);

//Chip release
void BAR_chipRelease(BAR* BAR);

//Read register from barometer
HAL_StatusTypeDef BAR_read(BAR* BAR, uint8_t reg_addr, uint8_t* rx_buffer, uint8_t num_bytes);

//Write register from barometer
HAL_StatusTypeDef BAR_write(BAR* BAR, uint8_t* tx_buffer, uint8_t num_bytes);

//Initialize barometer
int BAR_init(BAR* BAR);

//Get pressure from barometer
int BAR_getPres(BAR* BAR, float* pres);

//Get angular rate from barometer
int BAR_getAlt(BAR* BAR, float* alt, float BAR_SEA_LEVEL_PRESS);

//Get temperature from barometer
int BAR_getTemp(BAR* BAR, float* temp);

//Send command to barometer
int BAR_send(BAR* BAR, uint8_t cmd, uint8_t value);

#endif
