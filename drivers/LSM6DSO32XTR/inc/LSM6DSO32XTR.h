/*
 * LSM6DSO32XTR.h
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#ifndef LSM6DSO32XTR_H	// Begin header include protection
#define LSM6DSO32XTR_H

#define WHO_AM_I_REG_ADDR (uint8_t) 0x0F //Who am I register address
#define WHO_AM_I_REG_VAL (uint8_t) 0x6C //Who am I register value
#define OUT_TEMP_L (uint8_t) 0x20 //Temperature output register address
#define OUT_TEMP_H (uint8_t) 0x21 //Temperature output register address

#include <stm32h723xx.h> //I added this for intellisense to work, this might not be needed
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct {
    SPI_HandleTypeDef* hspi;
    uint16_t SPI_TIMEOUT;

    GPIO_TypeDef * CS_GPIO_Port;
    uint16_t CS_GPIO_Pin;
} IMU;

typedef struct {
    //Acceleration in g
    float accel_x;
    float accel_y;
    float accel_z;
} Accel;

typedef struct {
    //Angular Rate in deg/sec
    float ang_rate_x;
    float ang_rate_y;
    float ang_rate_z;
} AngRate;

//Temp convert
float IMUfloatConvert(uint8_t H_Byte, uint8_t L_Byte)

//Chip select
void IMUchipSelect(IMU* IMU);

//Chip release
void IMUchipRelease(IMU* IMU);

//Read register from IMU
HAL_StatusTypeDef IMUread(IMU* IMU, uint8_t reg_addr, uint8_t rx_buffer, uint8_t num_bytes);

//Write register from IMU
HAL_StatusTypeDef IMUwrite(IMU* IMU, uint8_t tx_buffer, uint8_t num_bytes);

//Initialize IMU
int IMUinit(SPI_HandleTypeDef* hspi, IMU* IMU, GPIO_TypeDef * CS_GPIO_Port, uint16_t CS_GPIO_Pin, uint16_t SPI_TIMEOUT);

//Get acceleration from IMU
int IMUgetAccel(IMU* IMU, Accel* accel);

//Get angular rate from IMU
int IMUgetAngRate(IMU* IMU, AngRate* AngRate);

//Get temperature from IMU
int IMUgetTemp(IMU* IMU, float* temp);

//Send command to IMU
int IMUsend();

#endif    // End header include protection
