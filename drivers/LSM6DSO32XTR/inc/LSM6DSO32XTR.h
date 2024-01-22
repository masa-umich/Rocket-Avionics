/*
 * LSM6DSO32XTR.h
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#ifndef LSM6DSO32XTR_H	// Begin header include protection
#define LSM6DSO32XTR_H

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

//Initialize IMU
int IMUinit(SPI_HandleTypeDef* hspi1, int timeout);

//Send command to IMU
int IMUsend();

//Read register from IMU
int IMUread();

//Write register from IMU
int IMUwrite();

//Get acceleration from IMU
int IMUgetAccel();

//Get angular rate from IMU
int IMUgetAngRate();

//Get temperature from IMU
int IMUgetTemp();

#endif    // End header include protection
