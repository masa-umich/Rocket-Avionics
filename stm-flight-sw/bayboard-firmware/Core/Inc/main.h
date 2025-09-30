/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "LSM6DSO32XTR.h"
//#include "semphr.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define BAY_BOARD

#define TCP_PORT			5000

#define TELEMETRY_HZ		(uint32_t)50

#define BB_EEPROM_LEN		(uint16_t)145
//#define EEPROM_OVERRIDE // Override EEPROM configuration. Use this when setting the EEPROM config on a board for the first time

#define BB_EEPROM_PT_ZERO_DEFAULT			0.5
#define BB_EEPROM_PT_RANGE_DEFAULT			5000
#define BB_EEPROM_PT_MAX_DEFAULT			4.5
#define BB_EEPROM_TC_GAIN_DEFAULT			0x00
#define BB_EEPROM_VLV_VOL_DEFAULT			0x01
#define BB_EEPROM_VLV_EN_DEFAULT			0x01

#define BB_EEPROM_FCIP_DEFAULT_1			141
#define BB_EEPROM_FCIP_DEFAULT_2			212
#define BB_EEPROM_FCIP_DEFAULT_3			192
#define BB_EEPROM_FCIP_DEFAULT_4			170

#define BB_EEPROM_BBIP_DEFAULT_1			141
#define BB_EEPROM_BBIP_DEFAULT_2			212
#define BB_EEPROM_BBIP_DEFAULT_3			192
#define BB_EEPROM_BBIP_DEFAULT_4			180

#define BB1_MAC_ADDR_1		0x00
#define BB1_MAC_ADDR_2		0x80
#define BB1_MAC_ADDR_3		0xE1
#define BB1_MAC_ADDR_4		0x93
#define BB1_MAC_ADDR_5		0x4E
#define BB1_MAC_ADDR_6		0xB6

#define BB2_MAC_ADDR_1		0x00
#define BB2_MAC_ADDR_2		0x80
#define BB2_MAC_ADDR_3		0xE1
#define BB2_MAC_ADDR_4		0xC3
#define BB2_MAC_ADDR_5		0x2A
#define BB2_MAC_ADDR_6		0x52

#define BB3_MAC_ADDR_1		0x00
#define BB3_MAC_ADDR_2		0x80
#define BB3_MAC_ADDR_3		0xE1
#define BB3_MAC_ADDR_4		0x8C
#define BB3_MAC_ADDR_5		0x19
#define BB3_MAC_ADDR_6		0xA1

#define FLASH_TELEM_MARK	(uint8_t)0x1D
#define FLASH_MSG_MARK		(uint8_t)0x1E

#define ERROR_UDP_PORT		(uint16_t)1234

#define ERROR_MSG_TYPES		(size_t)32 // Should be a multiple of 2
#define PERI_ERROR_MSG_TYPES	(size_t)10

#define ERROR_THROTTLE_MAX		10 // Not more than 15

#define SNTP_SET_SYSTEM_TIME_US(sec, us) set_system_time(sec, us)

#define NUM_TCS				6
#define NUM_SOLENOIDS		5
#define NUM_VALVE_CHS		7

#define SHUNT_RES_20M		(uint16_t)20
#define SHUNT_RES_2M		(uint16_t)2
#define VALVE_SHUNT_RES		SHUNT_RES_20M
#define POWER_SHUNT_3V3_5V	SHUNT_RES_20M
#define POWER_SHUNT_12V_24V SHUNT_RES_2M

#define POWER_24V_RES_A		(uint32_t)100000
#define POWER_24V_RES_B		(uint32_t)14300
#define POWER_12V_RES_A		(uint32_t)100000
#define POWER_12V_RES_B		(uint32_t)33200
#define POWER_5V_RES_A		(uint32_t)10000
#define POWER_5V_RES_B		(uint32_t)15000
#define POWER_3V3_RES_A		(uint32_t)1000
#define POWER_3V3_RES_B		(uint32_t)10000

#define ADC1_3V3_BUS_I		(size_t)1
#define ADC1_VLV1_CURRENT_I	(size_t)8
#define ADC1_VLV2_CURRENT_I	(size_t)9
#define ADC1_VLV3_CURRENT_I	(size_t)10
#define ADC1_VLV4_CURRENT_I	(size_t)11
#define ADC1_VLV5_CURRENT_I	(size_t)12
#define ADC1_24V_CURRENT_I	(size_t)7
#define ADC1_12V_CURRENT_I	(size_t)5
#define ADC1_5V_CURRENT_I	(size_t)3
#define ADC1_3V3_CURRENT_I	(size_t)0
#define ADC1_5V_BUS_I		(size_t)2
#define ADC1_12V_BUS_I		(size_t)4
#define ADC1_24V_BUS_I		(size_t)6

#define ADC2_PT1_I			(size_t)12
#define ADC2_PT2_I			(size_t)10
#define ADC2_PT3_I			(size_t)8
#define ADC2_PT4_I			(size_t)7
#define ADC2_PT5_I			(size_t)13
#define ADC2_PT6_I			(size_t)11
#define ADC2_PT7_I			(size_t)9
#define ADC2_PT8_I			(size_t)6
#define ADC2_PT9_I			(size_t)5
#define ADC2_PT10_I			(size_t)14
#define ADC2_HBRIDGE_I		(size_t)3

#define DIVIDER_VALVE		(uint8_t)1
#define DIVIDER_12V_24V		(uint8_t)0
#define DIVIDER_3V3_5V		(uint8_t)1

#define PERIPHERAL_TIMEOUT	1 // No reason to delay, plus being stuck in a HAL timeout is bad because we turn off interrupts during HAL API calls
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void set_system_time(uint32_t sec, uint32_t us);

uint8_t log_message(const char *msgtext, int msgtype);
uint8_t log_peri_message(const char *msgtext, int msgtype);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC1_CS_Pin GPIO_PIN_4
#define ADC1_CS_GPIO_Port GPIOE
#define ENCODER_CS_Pin GPIO_PIN_6
#define ENCODER_CS_GPIO_Port GPIOF
#define BAR2_CS_Pin GPIO_PIN_3
#define BAR2_CS_GPIO_Port GPIOA
#define BAR1_CS_Pin GPIO_PIN_4
#define BAR1_CS_GPIO_Port GPIOA
#define ETH_NRST_Pin GPIO_PIN_0
#define ETH_NRST_GPIO_Port GPIOB
#define ADC2_CS_Pin GPIO_PIN_11
#define ADC2_CS_GPIO_Port GPIOE
#define VLV5_OLD_Pin GPIO_PIN_12
#define VLV5_OLD_GPIO_Port GPIOD
#define VLV4_OLD_Pin GPIO_PIN_13
#define VLV4_OLD_GPIO_Port GPIOD
#define VLV3_OLD_Pin GPIO_PIN_14
#define VLV3_OLD_GPIO_Port GPIOD
#define VLV2_OLD_Pin GPIO_PIN_15
#define VLV2_OLD_GPIO_Port GPIOD
#define VLV1_OLD_Pin GPIO_PIN_6
#define VLV1_OLD_GPIO_Port GPIOG
#define VLV5_EN_Pin GPIO_PIN_7
#define VLV5_EN_GPIO_Port GPIOG
#define VLV4_EN_Pin GPIO_PIN_8
#define VLV4_EN_GPIO_Port GPIOG
#define VLV3_EN_Pin GPIO_PIN_6
#define VLV3_EN_GPIO_Port GPIOC
#define VLV2_EN_Pin GPIO_PIN_7
#define VLV2_EN_GPIO_Port GPIOC
#define VLV1_EN_Pin GPIO_PIN_8
#define VLV1_EN_GPIO_Port GPIOC
#define BUFF_CLK_Pin GPIO_PIN_10
#define BUFF_CLK_GPIO_Port GPIOA
#define TC1_CS_Pin GPIO_PIN_11
#define TC1_CS_GPIO_Port GPIOA
#define VLV_CTRL_Pin GPIO_PIN_15
#define VLV_CTRL_GPIO_Port GPIOA
#define BUZZ_Pin GPIO_PIN_10
#define BUZZ_GPIO_Port GPIOC
#define LED_BLUE_Pin GPIO_PIN_11
#define LED_BLUE_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_0
#define LED_RED_GPIO_Port GPIOD
#define IMU1_INT1_Pin GPIO_PIN_1
#define IMU1_INT1_GPIO_Port GPIOD
#define IMU1_INT2_Pin GPIO_PIN_2
#define IMU1_INT2_GPIO_Port GPIOD
#define IMU2_INT1_Pin GPIO_PIN_4
#define IMU2_INT1_GPIO_Port GPIOD
#define IMU2_INT2_Pin GPIO_PIN_5
#define IMU2_INT2_GPIO_Port GPIOD
#define FLASH_CS_Pin GPIO_PIN_10
#define FLASH_CS_GPIO_Port GPIOG
#define TC2_CS_Pin GPIO_PIN_4
#define TC2_CS_GPIO_Port GPIOB
#define EEPROM_WC_Pin GPIO_PIN_5
#define EEPROM_WC_GPIO_Port GPIOB
#define TC3_CS_Pin GPIO_PIN_9
#define TC3_CS_GPIO_Port GPIOB
#define HBRIDGE_EN_Pin GPIO_PIN_0
#define HBRIDGE_EN_GPIO_Port GPIOE
#define HBRIDGE_DIR_Pin GPIO_PIN_1
#define HBRIDGE_DIR_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
