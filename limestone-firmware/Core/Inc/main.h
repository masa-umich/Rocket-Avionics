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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

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
