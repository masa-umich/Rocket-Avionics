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
#define FLIGHT_COMPUTER

#define TCP_PORT			5000

#define TELEMETRY_HZ		(uint32_t)50

#define FC_EEPROM_LEN		(uint16_t)93
//#define EEPROM_OVERRIDE // Override EEPROM configuration. Use this when setting the EEPROM config on a board for the first time

#define FC_EEPROM_PT_ZERO_DEFAULT			0.5
#define FC_EEPROM_PT_RANGE_DEFAULT			5000
#define FC_EEPROM_PT_MAX_DEFAULT			4.5
#define FC_EEPROM_TC_GAIN_DEFAULT			0x00
#define FC_EEPROM_VLV_VOL_DEFAULT			0x01
#define FC_EEPROM_VLV_EN_DEFAULT			0x01

#define FC_EEPROM_LIMEWIREIP_DEFAULT_1		141
#define FC_EEPROM_LIMEWIREIP_DEFAULT_2		212
#define FC_EEPROM_LIMEWIREIP_DEFAULT_3		192
#define FC_EEPROM_LIMEWIREIP_DEFAULT_4		160

#define FC_EEPROM_FCIP_DEFAULT_1			141
#define FC_EEPROM_FCIP_DEFAULT_2			212
#define FC_EEPROM_FCIP_DEFAULT_3			192
#define FC_EEPROM_FCIP_DEFAULT_4			170

#define FC_EEPROM_BB1IP_DEFAULT_1			141
#define FC_EEPROM_BB1IP_DEFAULT_2			212
#define FC_EEPROM_BB1IP_DEFAULT_3			192
#define FC_EEPROM_BB1IP_DEFAULT_4			180

#define FC_EEPROM_BB2IP_DEFAULT_1			141
#define FC_EEPROM_BB2IP_DEFAULT_2			212
#define FC_EEPROM_BB2IP_DEFAULT_3			192
#define FC_EEPROM_BB2IP_DEFAULT_4			190

#define FC_EEPROM_BB3IP_DEFAULT_1			141
#define FC_EEPROM_BB3IP_DEFAULT_2			212
#define FC_EEPROM_BB3IP_DEFAULT_3			192
#define FC_EEPROM_BB3IP_DEFAULT_4			200

#define FC_EEPROM_FRIP_DEFAULT_1			141
#define FC_EEPROM_FRIP_DEFAULT_2			212
#define FC_EEPROM_FRIP_DEFAULT_3			192
#define FC_EEPROM_FRIP_DEFAULT_4			210

#define FC_MAC_ADDR_1		0x00
#define FC_MAC_ADDR_2		0x80
#define FC_MAC_ADDR_3		0xE1
#define FC_MAC_ADDR_4		0x3f
#define FC_MAC_ADDR_5		0xd8
#define FC_MAC_ADDR_6		0xa2

#define FLASH_TELEM_MARK	(uint8_t)0x1D
#define FLASH_MSG_MARK		(uint8_t)0x1E

#define ERROR_UDP_PORT		(uint16_t)1234

#define ERROR_MSG_TYPES		(size_t)32 // Should be a multiple of 2
#define PERI_ERROR_MSG_TYPES	(size_t)8

#define ERROR_THROTTLE_MAX		10 // Not more than 15

#define SNTP_SET_SYSTEM_TIME_US(sec, us) set_system_time(sec, us)

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

#define ADC_3V3_BUS_I		(size_t)0
#define ADC_VLV1_CURRENT_I	(size_t)1
#define ADC_VLV2_CURRENT_I	(size_t)2
#define ADC_VLV3_CURRENT_I	(size_t)3
#define ADC_PT1_I			(size_t)4
#define ADC_PT2_I			(size_t)5
#define ADC_PT3_I			(size_t)6
#define ADC_PT4_I			(size_t)7
#define ADC_PT5_I			(size_t)8
#define ADC_24V_CURRENT_I	(size_t)9
#define ADC_12V_CURRENT_I	(size_t)10
#define ADC_5V_CURRENT_I	(size_t)11
#define ADC_3V3_CURRENT_I	(size_t)12
#define ADC_5V_BUS_I		(size_t)13
#define ADC_12V_BUS_I		(size_t)14
#define ADC_24V_BUS_I		(size_t)15

#define DIVIDER_VALVE		(uint8_t)1
#define DIVIDER_12V_24V		(uint8_t)0
#define DIVIDER_3V3_5V		(uint8_t)1

#define PERIPHERAL_TIMEOUT		1
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void set_system_time(uint32_t sec, uint32_t us);

uint8_t log_message(const char *msgtext, int msgtype);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPS_NRST_Pin GPIO_PIN_4
#define GPS_NRST_GPIO_Port GPIOE
#define RF_DIO3_Pin GPIO_PIN_13
#define RF_DIO3_GPIO_Port GPIOC
#define RF_DIO2_Pin GPIO_PIN_14
#define RF_DIO2_GPIO_Port GPIOC
#define RF_DIO1_Pin GPIO_PIN_15
#define RF_DIO1_GPIO_Port GPIOC
#define RADIO_CS_Pin GPIO_PIN_6
#define RADIO_CS_GPIO_Port GPIOF
#define RF_BUSY_Pin GPIO_PIN_10
#define RF_BUSY_GPIO_Port GPIOF
#define RF_MODE_SW_Pin GPIO_PIN_0
#define RF_MODE_SW_GPIO_Port GPIOC
#define RF_NSRT_Pin GPIO_PIN_2
#define RF_NSRT_GPIO_Port GPIOC
#define BAR2_CS_Pin GPIO_PIN_3
#define BAR2_CS_GPIO_Port GPIOA
#define BAR1_CS_Pin GPIO_PIN_4
#define BAR1_CS_GPIO_Port GPIOA
#define ETH_NRST_Pin GPIO_PIN_0
#define ETH_NRST_GPIO_Port GPIOB
#define PDB_DIO1_Pin GPIO_PIN_14
#define PDB_DIO1_GPIO_Port GPIOF
#define PDB_DIO2_Pin GPIO_PIN_15
#define PDB_DIO2_GPIO_Port GPIOF
#define ADC_CS_Pin GPIO_PIN_11
#define ADC_CS_GPIO_Port GPIOE
#define VLV2_OLD_Pin GPIO_PIN_15
#define VLV2_OLD_GPIO_Port GPIOE
#define TC1_CS_Pin GPIO_PIN_9
#define TC1_CS_GPIO_Port GPIOD
#define TC2_CS_Pin GPIO_PIN_10
#define TC2_CS_GPIO_Port GPIOD
#define VLV3_EN_Pin GPIO_PIN_8
#define VLV3_EN_GPIO_Port GPIOG
#define VLV1_EN_Pin GPIO_PIN_6
#define VLV1_EN_GPIO_Port GPIOC
#define VLV3_OLD_Pin GPIO_PIN_7
#define VLV3_OLD_GPIO_Port GPIOC
#define VLV1_OLD_Pin GPIO_PIN_8
#define VLV1_OLD_GPIO_Port GPIOC
#define VLV2_EN_Pin GPIO_PIN_8
#define VLV2_EN_GPIO_Port GPIOA
#define BUFF_CLR_Pin GPIO_PIN_9
#define BUFF_CLR_GPIO_Port GPIOA
#define BUFF_CLK_Pin GPIO_PIN_10
#define BUFF_CLK_GPIO_Port GPIOA
#define VLV_CTRL_Pin GPIO_PIN_15
#define VLV_CTRL_GPIO_Port GPIOA
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
#define EEPROM_WC_Pin GPIO_PIN_11
#define EEPROM_WC_GPIO_Port GPIOG
#define BUZZ_Pin GPIO_PIN_8
#define BUZZ_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOE
#define LED_BLUE_Pin GPIO_PIN_1
#define LED_BLUE_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
