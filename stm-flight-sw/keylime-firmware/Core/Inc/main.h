/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "semphr.h"
#include "stdlib.h"
#include "string.h"
#include "timers.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define FLIGHT_RECORDER

#define TCP_PORT			5000

#define TELEMETRY_HZ		(uint32_t)50

#define FR_EEPROM_LEN		(uint16_t)8
//#define EEPROM_OVERRIDE // Override EEPROM configuration. Use this when setting the EEPROM config on a board for the first time
#define RESTART_AFTER_CONFIG

#define EEPROM_RESTART_DELAY_MS				500

#define FR_EEPROM_FCIP_DEFAULT_1			141
#define FR_EEPROM_FCIP_DEFAULT_2			212
#define FR_EEPROM_FCIP_DEFAULT_3			192
#define FR_EEPROM_FCIP_DEFAULT_4			170

#define FR_EEPROM_FRIP_DEFAULT_1			141
#define FR_EEPROM_FRIP_DEFAULT_2			212
#define FR_EEPROM_FRIP_DEFAULT_3			192
#define FR_EEPROM_FRIP_DEFAULT_4			210

#define FR_MAC_ADDR_1		0x00
#define FR_MAC_ADDR_2		0x80
#define FR_MAC_ADDR_3		0xE1
#define FR_MAC_ADDR_4		0x3E
#define FR_MAC_ADDR_5		0x19
#define FR_MAC_ADDR_6		0xC2

#define FLASH_TELEM_MARK	(uint8_t)0x1D
#define FLASH_MSG_MARK		(uint8_t)0x1E
#define FLASH_BOTH_MARK		(uint8_t)0x1F

#define ERROR_UDP_PORT		(uint16_t)1234
#define TELEM_UDP_PORT		(uint16_t)6767

#define ERROR_MSG_TYPES		(size_t)32 // Should be a multiple of 2
#define PERI_ERROR_MSG_TYPES	(size_t)10

#define ERROR_THROTTLE_MAX		10 // Not more than 15

extern void set_system_time(uint32_t sec, uint32_t us);
#define SNTP_SET_SYSTEM_TIME_US(sec, us) set_system_time(sec, us)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI4_CS_Pin GPIO_PIN_4
#define SPI4_CS_GPIO_Port GPIOE
#define SPI5_CS_Pin GPIO_PIN_6
#define SPI5_CS_GPIO_Port GPIOF
#define SPI2_CS_Pin GPIO_PIN_11
#define SPI2_CS_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_10
#define SPI1_CS_GPIO_Port GPIOG
#define EEPROM_WC_Pin GPIO_PIN_12
#define EEPROM_WC_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
