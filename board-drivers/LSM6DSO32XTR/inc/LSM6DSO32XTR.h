/*
 * LSM6DSO32XTR.h
 *
 *  Created on: January 21, 2024
 *      Author: jackmh
 */

#ifndef LSM6DSO32XTR_H	// Begin header include protection
#define LSM6DSO32XTR_H

#include "main.h"

#ifndef FreeRTOS_H
    #include "FreeRTOS.h"
    #include "task.h"
#else
    #error "This library is designed for use with FreeRTOS. Please include the FreeRTOS library in your project."
#endif

// Begin register map
#define IMU_I2C_ADDR_BASE				(uint8_t)(0x35 << 2) //Device Address base
#define IMU_FUNC_CFG_ACCESS             (uint8_t)0x01 //R/W - Function configuration access register address
#define IMU_PIN_CTRL                    (uint8_t)0x02 //R/W - Pin control register address
#define IMU_FIFO_CTRL1                  (uint8_t)0x07 //R/W - FIFO control register 1 address
#define IMU_FIFO_CTRL2                  (uint8_t)0x08 //R/W - FIFO control register 2 address
#define IMU_FIFO_CTRL3                  (uint8_t)0x09 //R/W - FIFO control register 3 address
#define IMU_FIFO_CTRL4                  (uint8_t)0x0A //R/W - FIFO control register 4 address
#define IMU_COUNTER_BDR_REG1            (uint8_t)0x0B //R/W - Counter and BDR register 1 address
#define IMU_COUNTER_BDR_REG2            (uint8_t)0x0C //R/W - Counter and BDR register 2 address
#define IMU_INT1_CTRL                   (uint8_t)0x0D //R/W - Interrupt 1 control register address
#define IMU_INT2_CTRL                   (uint8_t)0x0E //R/W - Interrupt 2 control register address
#define IMU_WHO_AM_I_REG_ADDR           (uint8_t)0x0F //R - Who am I register address
#define IMU_CTRL1_XL                    (uint8_t)0x10 //R/W - Control 1 accelerometer register address
#define IMU_CTRL2_G                     (uint8_t)0x11 //R/W - Control 2 gyroscope register address
#define IMU_CTRL3_C                     (uint8_t)0x12 //R/W - Control 3 register address
#define IMU_CTRL4_C                     (uint8_t)0x13 //R/W - Control 4 register address
#define IMU_CTRL5_C                     (uint8_t)0x14 //R/W - Control 5 register address
#define IMU_CTRL6_C                     (uint8_t)0x15 //R/W - Control 6 register address
#define IMU_CTRL7_G                     (uint8_t)0x16 //R/W - Control 7 register address
#define IMU_CTRL8_XL                    (uint8_t)0x17 //R/W - Control 8 accelerometer register address
#define IMU_CTRL9_XL                    (uint8_t)0x18 //R/W - Control 9 accelerometer register address
#define IMU_CTRL10_C                    (uint8_t)0x19 //R/W - Control 10 register address
#define IMU_ALL_INT_SRC                 (uint8_t)0x1A //R - All interrupt source register address
#define IMU_WAKE_UP_SRC                 (uint8_t)0x1B //R - Wake up interrupt source register address
#define IMU_TAP_SRC                     (uint8_t)0x1C //R - Tap interrupt source register address
#define IMU_D6D_SRC                     (uint8_t)0x1D //R - 6D interrupt source register address
#define IMU_STATUS_REG                  (uint8_t)0x1E //R - Status register address
#define IMU_OUT_TEMP_L                  (uint8_t)0x20 //R - Temperature output register address
#define IMU_OUT_TEMP_H                  (uint8_t)0x21 //R - Temperature output register address
#define IMU_OUTX_L_G                    (uint8_t)0x22 //R - Angular rate X output register address
#define IMU_OUTX_H_G                    (uint8_t)0x23 //R - Angular rate X output register address
#define IMU_OUTY_L_G                    (uint8_t)0x24 //R - Angular rate Y output register address
#define IMU_OUTY_H_G                    (uint8_t)0x25 //R - Angular rate Y output register address
#define IMU_OUTZ_L_G                    (uint8_t)0x26 //R - Angular rate Z output register address
#define IMU_OUTZ_H_G                    (uint8_t)0x27 //R - Angular rate Z output register address
#define IMU_OUTX_L_A                    (uint8_t)0x28 //R - Acceleration X output register address
#define IMU_OUTX_H_A                    (uint8_t)0x29 //R - Acceleration X output register address
#define IMU_OUTY_L_A                    (uint8_t)0x2A //R - Acceleration Y output register address
#define IMU_OUTY_H_A                    (uint8_t)0x2B //R - Acceleration Y output register address
#define IMU_OUTZ_L_A                    (uint8_t)0x2C //R - Acceleration Z output register address
#define IMU_OUTZ_H_A                    (uint8_t)0x2D //R - Acceleration Z output register address
#define IMU_EMB_FUNC_STATUS_MAINPAGE    (uint8_t)0x35 //R - Embedded function status main page register address
#define IMU_FSM_STATUS_A_MAINPAGE       (uint8_t)0x36 //R - Finite state machine status accelerometer main page register address
#define IMU_FSM_STATUS_B_MAINPAGE       (uint8_t)0x37 //R - Finite state machine status gyroscope main page register address
#define IMU_MLC_STATUS_MAINPAGE         (uint8_t)0x38 //R - Machine learning core status main page register address
#define IMU_STATUS_MASTER_MAINPAGE      (uint8_t)0x39 //R - Status master main page register address
#define IMU_FIFO_STATUS1                (uint8_t)0x3A //R - FIFO status register 1 address
#define IMU_FIFO_STATUS2                (uint8_t)0x3B //R - FIFO status register 2 address
#define IMU_TEMESTAMP0                  (uint8_t)0x40 //R - Timestamp register 0 address
#define IMU_TEMESTAMP1                  (uint8_t)0x41 //R - Timestamp register 1 address
#define IMU_TEMESTAMP2                  (uint8_t)0x42 //R - Timestamp register 2 address
#define IMU_TIMESTAMP3                  (uint8_t)0x43 //R - Timestamp register 3 address
#define IMU_TAP_CFG0                    (uint8_t)0x56 //R/W - Tap configuration register 0 address
#define IMU_TAP_CFG1                    (uint8_t)0x57 //R/W - Tap configuration register 1 address
#define IMU_TAP_CFG2                    (uint8_t)0x58 //R/W - Tap configuration register 2 address
#define IMU_TAP_THS_6D                  (uint8_t)0x59 //R/W - Tap threshold 6D register address
#define IMU_INT_DUR2                    (uint8_t)0x5A //R/W - Tap duration register address
#define IMU_WAKE_UP_THS                 (uint8_t)0x5B //R/W - Wake up threshold register address
#define IMU_WAKE_UP_DUR                 (uint8_t)0x5C //R/W - Wake up duration register address
#define IMU_FREE_FALL                   (uint8_t)0x5D //R/W - Free fall duration register address
#define IMU_MD1_CFG                     (uint8_t)0x5E //R/W - Interrupt 1 configuration register address
#define IMU_MD2_CFG                     (uint8_t)0x5F //R/W - Interrupt 2 configuration register address
#define IMU_I3C_BUS_AVB                 (uint8_t)0x62 //R/W - I3C bus availability register address
#define IMU_INTERNAL_FREQ_FINE          (uint8_t)0x63 //R - Internal frequency fine register address
#define IMU_X_OFS_USR                   (uint8_t)0x73 //R/W - Accelerometer X axis user offset register address
#define IMU_Y_OFS_USR                   (uint8_t)0x74 //R/W - Accelerometer Y axis user offset register address
#define IMU_Z_OFS_USR                   (uint8_t)0x75 //R/W - Accelerometer Z axis user offset register address
#define IMU_FIFO_DATA_OUT_TAG           (uint8_t)0x78 //R - FIFO data output tag register address
#define IMU_FIFO_DATA_OUT_X_L           (uint8_t)0x79 //R - FIFO data output X register address
#define IMU_FIFO_DATA_OUT_X_H           (uint8_t)0x7A //R - FIFO data output X register address
#define IMU_FIFO_DATA_OUT_Y_L           (uint8_t)0x7B //R - FIFO data output Y register address
#define IMU_FIFO_DATA_OUT_Y_H           (uint8_t)0x7C //R - FIFO data output Y register address
#define IMU_FIFO_DATA_OUT_Z_L           (uint8_t)0x7D //R - FIFO data output Z register address
#define IMU_FIFO_DATA_OUT_Z_H           (uint8_t)0x7E //R - FIFO data output Z register address
// End register map

// Constants
#define IMU_g                           (float)9.80665 //Acceleration due to gravity in m/s^2
#define IMU_WHO_AM_I_REG_VAL            (uint8_t)0x6C //Who am I register value
#define IMU_FULLSCALE_ACCEL             (uint8_t)4 //Full scale acceleration in g 
#define IMU_FULLSCALE_GYRO              (uint16_t)2000 //Full scale angular rate in deg/sec
#define IMU_FULLSCALE_TEMP              (uint8_t)125 //Full scale temperature (-40 to 85 deg C)
#define IMU_RESOLUTION_ACCEL            (uint16_t)32767 //the maximum value of a signed 16 bit integer in 2's compliment
#define IMU_RESOLUTION_GYRO             (uint16_t)32767 //the maximum value of a signed 16 bit integer in 2's compliment
#define IMU_RESOLUTION_TEMP             (uint16_t)65535 //the maximum value of an unsigned 16 bit integer (adc resolution)
#define IMU_SCALING_FACTOR_ACCEL        (float)IMU_FULLSCALE_ACCEL/32767 //Scaling factor for acceleration (g/Accel res)
#define IMU_SCALING_FACTOR_GYRO         (float)IMU_FULLSCALE_GYRO/32767 //Scaling factor for angular rate (deg/sec/Gyro res)
#define IMU_SCALING_FACTOR_TEMP         (float)IMU_FULLSCALE_TEMP/65535 //Scaling factor for temperature (deg C/Temp ADC res)

/*
* Note: These default configs are a bit arbitrary. 
* I chose the + or - 4g range because according to the Limelight Master Sheet
* (Specifically the MASTRAN portion) max acceleration should be around 2.5g
* So for resolution sake, I figure it's best to go with close to double that value.
*
* As for the gyro, I just went with the highest DPS I could, which is 2000dps.
* This would be about 5.5 rpm and ideally, our rocket shouldn't be spinning much anyway
*
* Finally, for polling rate, although some things like ADCs, I've been told we should be able
* to sample at somewhere around 200hz (this target has shifted around a bit), not all sensors need
* a polling rate nearly that high. But there's no real disadvantage to having a higher polling rate
* and it allows for better future proofing and more flexibility in the future. I guess you could also
* oversample and average, but I'm not going to implement that here.
*
* If you want to find other configuration values for the IMU, check out the datasheet
*/
#define IMU_DEFAULT_CONF_ACCEL          (uint8_t)0x50 // 01010000 208hz, + or - 4g range
#define IMU_DEFAULT_CONF_GYRO           (uint8_t)0x5C // 01011100 208hz, + or - 2000dps range
#define IMU_DEFAULT_CONF_CTRL3_C		(uint8_t)0x04 // 00000100 Main control register - register shifting is set to true
//Otther default configs
#define IMU_DEFAULT_CONF_CTRL4_C        (uint8_t)0x00
#define IMU_DEFAULT_CONF_CTRL5_C        (uint8_t)0x00
#define IMU_DEFAULT_CONF_CTRL6_C        (uint8_t)0x10 // 00010000
#define IMU_DEFAULT_CONF_CTRL7_G        (uint8_t)0x80 // 10000000
#define IMU_DEFAULT_CONF_CTRL8_XL       (uint8_t)0x00
#define IMU_DEFAULT_CONF_CTRL9_XL       (uint8_t)0x00
#define IMU_DEFAULT_CONF_CTRL10_C       (uint8_t)0x00 // Disables timestamps
#define IMU_SW_RESET_CONF_CTRL3_C		(uint8_t)0x05 // 00001001 This will reset the device (ik right who could've guessed)

typedef struct {
    //I2C stuff
	I2C_HandleTypeDef* hi2c;
    uint16_t I2C_TIMEOUT;
    uint8_t SA0; // Changes I2C address

    //Offset values
    float XL_x_offset;
    float XL_y_offset;
    float XL_z_offset;
    float G_x_offset;
    float G_y_offset;
    float G_z_offset;
} IMU;

typedef struct {
    //Acceleration in g
    float XL_x;
    float XL_y;
    float XL_z;
} Accel;

typedef struct {
    //Angular Rate in deg/sec
    float G_x;
    float G_y;
    float G_z;
} AngRate;

//Temp convert
float IMU_tempConvert(uint8_t H_Byte, uint8_t L_Byte);

//Accel convert
float IMU_accelConvert(uint8_t H_Byte, uint8_t L_Byte);

//Gyro convert
float IMU_gyroConvert(uint8_t H_Byte, uint8_t L_Byte);

//Chip select
void IMU_chipSelect(IMU* IMU);

//Chip release
void IMU_chipRelease(IMU* IMU);

//Read register from IMU
HAL_StatusTypeDef IMU_read(IMU* IMU, uint8_t reg_addr, uint8_t* rx_buffer, uint8_t num_bytes);

//Write register from IMU
HAL_StatusTypeDef IMU_write(IMU* IMU, uint8_t* tx_buffer, uint8_t num_bytes);

//Initialize IMU
int IMU_init(IMU* IMU);

//Get acceleration from IMU
int IMU_getAccel(IMU* IMU, Accel* accel);

//Get angular rate from IMU
int IMU_getAngRate(IMU* IMU, AngRate* AngRate);

//Get temperature from IMU
int IMU_getTemp(IMU* IMU, float* temp);

//Send command to IMU
int IMU_send(IMU* IMU, uint8_t cmd, uint8_t value);

#endif    // End header include protection
