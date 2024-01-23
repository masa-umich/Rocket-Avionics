/*
 * LSM6DSO32XTR.h
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#ifndef LSM6DSO32XTR_H	// Begin header include protection
#define LSM6DSO32XTR_H

#include <stm32h723xx.h> //I added this for intellisense to work, this might not be needed
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

// Begin register map
#define FUNC_CFG_ACCESS             (uint8_t)0x01 //R/W - Function configuration access register address
#define PIN_CTRL                    (uint8_t)0x02 //R/W - Pin control register address
#define FIFO_CTRL1                  (uint8_t)0x07 //R/W - FIFO control register 1 address
#define FIFO_CTRL2                  (uint8_t)0x08 //R/W - FIFO control register 2 address
#define FIFO_CTRL3                  (uint8_t)0x09 //R/W - FIFO control register 3 address
#define FIFO_CTRL4                  (uint8_t)0x0A //R/W - FIFO control register 4 address
#define COUNTER_BDR_REG1            (uint8_t)0x0B //R/W - Counter and BDR register 1 address
#define COUNTER_BDR_REG2            (uint8_t)0x0C //R/W - Counter and BDR register 2 address
#define INT1_CTRL                   (uint8_t)0x0D //R/W - Interrupt 1 control register address
#define INT2_CTRL                   (uint8_t)0x0E //R/W - Interrupt 2 control register address
#define WHO_AM_I_REG_ADDR           (uint8_t)0x0F //R - Who am I register address
#define CTRL1_XL                    (uint8_t)0x10 //R/W - Control 1 accelerometer register address
#define CTRL2_G                     (uint8_t)0x11 //R/W - Control 2 gyroscope register address
#define CTRL3_C                     (uint8_t)0x12 //R/W - Control 3 register address
#define CTRL4_C                     (uint8_t)0x13 //R/W - Control 4 register address
#define CTRL5_C                     (uint8_t)0x14 //R/W - Control 5 register address
#define CTRL6_C                     (uint8_t)0x15 //R/W - Control 6 register address
#define CTRL7_G                     (uint8_t)0x16 //R/W - Control 7 register address
#define CTRL8_XL                    (uint8_t)0x17 //R/W - Control 8 accelerometer register address
#define CTRL9_XL                    (uint8_t)0x18 //R/W - Control 9 accelerometer register address
#define CTRL10_C                    (uint8_t)0x19 //R/W - Control 10 register address
#define ALL_INT_SRC                 (uint8_t)0x1A //R - All interrupt source register address
#define WAKE_UP_SRC                 (uint8_t)0x1B //R - Wake up interrupt source register address
#define TAP_SRC                     (uint8_t)0x1C //R - Tap interrupt source register address
#define D6D_SRC                     (uint8_t)0x1D //R - 6D interrupt source register address
#define STATUS_REG                  (uint8_t)0x1E //R - Status register address
#define OUT_TEMP_L                  (uint8_t)0x20 //R - Temperature output register address
#define OUT_TEMP_H                  (uint8_t)0x21 //R - Temperature output register address
#define OUTX_L_G                    (uint8_t)0x22 //R - Angular rate X output register address
#define OUTX_H_G                    (uint8_t)0x23 //R - Angular rate X output register address
#define OUTY_L_G                    (uint8_t)0x24 //R - Angular rate Y output register address
#define OUTY_H_G                    (uint8_t)0x25 //R - Angular rate Y output register address
#define OUTZ_L_G                    (uint8_t)0x26 //R - Angular rate Z output register address
#define OUTZ_H_G                    (uint8_t)0x27 //R - Angular rate Z output register address
#define OUTX_L_A                    (uint8_t)0x28 //R - Acceleration X output register address
#define OUTX_H_A                    (uint8_t)0x29 //R - Acceleration X output register address
#define OUTY_L_A                    (uint8_t)0x2A //R - Acceleration Y output register address
#define OUTY_H_A                    (uint8_t)0x2B //R - Acceleration Y output register address
#define OUTZ_L_A                    (uint8_t)0x2C //R - Acceleration Z output register address
#define OUTZ_H_A                    (uint8_t)0x2D //R - Acceleration Z output register address
#define EMB_FUNC_STATUS_MAINPAGE    (uint8_t)0x35 //R - Embedded function status main page register address
#define FSM_STATUS_A_MAINPAGE       (uint8_t)0x36 //R - Finite state machine status accelerometer main page register address
#define FSM_STATUS_B_MAINPAGE       (uint8_t)0x37 //R - Finite state machine status gyroscope main page register address
#define MLC_STATUS_MAINPAGE         (uint8_t)0x38 //R - Machine learning core status main page register address
#define STATUS_MASTER_MAINPAGE      (uint8_t)0x39 //R - Status master main page register address
#define FIFO_STATUS1                (uint8_t)0x3A //R - FIFO status register 1 address
#define FIFO_STATUS2                (uint8_t)0x3B //R - FIFO status register 2 address
#define TEMESTAMP0                  (uint8_t)0x40 //R - Timestamp register 0 address
#define TEMESTAMP1                  (uint8_t)0x41 //R - Timestamp register 1 address
#define TEMESTAMP2                  (uint8_t)0x42 //R - Timestamp register 2 address
#define TIMESTAMP3                  (uint8_t)0x43 //R - Timestamp register 3 address
#define TAP_CFG0                    (uint8_t)0x56 //R/W - Tap configuration register 0 address
#define TAP_CFG1                    (uint8_t)0x57 //R/W - Tap configuration register 1 address
#define TAP_CFG2                    (uint8_t)0x58 //R/W - Tap configuration register 2 address
#define TAP_THS_6D                  (uint8_t)0x59 //R/W - Tap threshold 6D register address
#define INT_DUR2                    (uint8_t)0x5A //R/W - Tap duration register address
#define WAKE_UP_THS                 (uint8_t)0x5B //R/W - Wake up threshold register address
#define WAKE_UP_DUR                 (uint8_t)0x5C //R/W - Wake up duration register address
#define FREE_FALL                   (uint8_t)0x5D //R/W - Free fall duration register address
#define MD1_CFG                     (uint8_t)0x5E //R/W - Interrupt 1 configuration register address
#define MD2_CFG                     (uint8_t)0x5F //R/W - Interrupt 2 configuration register address
#define I3C_BUS_AVB                 (uint8_t)0x62 //R/W - I3C bus availability register address
#define INTERNAL_FREQ_FINE          (uint8_t)0x63 //R - Internal frequency fine register address
#define X_OFS_USR                   (uint8_t)0x73 //R/W - Accelerometer X axis user offset register address
#define Y_OFS_USR                   (uint8_t)0x74 //R/W - Accelerometer Y axis user offset register address
#define Z_OFS_USR                   (uint8_t)0x75 //R/W - Accelerometer Z axis user offset register address
#define FIFO_DATA_OUT_TAG           (uint8_t)0x78 //R - FIFO data output tag register address
#define FIFO_DATA_OUT_X_L           (uint8_t)0x79 //R - FIFO data output X register address
#define FIFO_DATA_OUT_X_H           (uint8_t)0x7A //R - FIFO data output X register address
#define FIFO_DATA_OUT_Y_L           (uint8_t)0x7B //R - FIFO data output Y register address
#define FIFO_DATA_OUT_Y_H           (uint8_t)0x7C //R - FIFO data output Y register address
#define FIFO_DATA_OUT_Z_L           (uint8_t)0x7D //R - FIFO data output Z register address
#define FIFO_DATA_OUT_Z_H           (uint8_t)0x7E //R - FIFO data output Z register address
// End register map

// Constants
#define g                           (float)9.80665 //Acceleration due to gravity in m/s^2
#define WHO_AM_I_REG_VAL            (uint8_t)0x6C //Who am I register value
#define FULLSCALE_ACCEL             (uint8_t)8 //Full scale acceleration in g 
#define FULLSCALE_GYRO              (uint16_t)2000 //Full scale angular rate in deg/sec
#define FULLSCALE_TEMP              (uint8_t)125 //Full scale temperature (-40 to 85 deg C)
#define SCALING_FACTOR_TEMP         (float)FULLSCALE_TEMP/65535 //Scaling factor for temperature (deg C/ADC res)

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
float IMUtempConvert(uint8_t H_Byte, uint8_t L_Byte);

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
