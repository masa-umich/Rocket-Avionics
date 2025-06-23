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
#define MODE_SWITCH_Pin GPIO_PIN_0
#define MODE_SWITCH_GPIO_Port GPIOC
#define SPI6_CS_Pin GPIO_PIN_4
#define SPI6_CS_GPIO_Port GPIOA
#define MCU_DIO3_Pin GPIO_PIN_15
#define MCU_DIO3_GPIO_Port GPIOD
#define MCU_DIO2_Pin GPIO_PIN_6
#define MCU_DIO2_GPIO_Port GPIOG
#define MCU_DIO1_Pin GPIO_PIN_7
#define MCU_DIO1_GPIO_Port GPIOG
#define SPI1_CS_Pin GPIO_PIN_10
#define SPI1_CS_GPIO_Port GPIOG
#define EEPROM_WC_Pin GPIO_PIN_11
#define EEPROM_WC_GPIO_Port GPIOG
#define MCU_LED_R_Pin GPIO_PIN_9
#define MCU_LED_R_GPIO_Port GPIOB
#define MCU_LED_G_Pin GPIO_PIN_0
#define MCU_LED_G_GPIO_Port GPIOE
#define MCU_LED_B_Pin GPIO_PIN_1
#define MCU_LED_B_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
