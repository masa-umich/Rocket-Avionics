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
#define SPI5_CS_Pin GPIO_PIN_6
#define SPI5_CS_GPIO_Port GPIOF
#define MODE_SW_Pin GPIO_PIN_0
#define MODE_SW_GPIO_Port GPIOC
#define RF_NSRT_Pin GPIO_PIN_2
#define RF_NSRT_GPIO_Port GPIOC
#define SPI6_CS_Pin GPIO_PIN_0
#define SPI6_CS_GPIO_Port GPIOA
#define BAR2_CS_Pin GPIO_PIN_3
#define BAR2_CS_GPIO_Port GPIOA
#define TC_CS1_Pin GPIO_PIN_9
#define TC_CS1_GPIO_Port GPIOD
#define TC_CS2_Pin GPIO_PIN_10
#define TC_CS2_GPIO_Port GPIOD
#define VLV_EN3_Pin GPIO_PIN_8
#define VLV_EN3_GPIO_Port GPIOG
#define VLV_EN1_Pin GPIO_PIN_6
#define VLV_EN1_GPIO_Port GPIOC
#define VLV_EN2_Pin GPIO_PIN_8
#define VLV_EN2_GPIO_Port GPIOA
#define BUFF_CLR_Pin GPIO_PIN_9
#define BUFF_CLR_GPIO_Port GPIOA
#define BUFF_CLK_Pin GPIO_PIN_10
#define BUFF_CLK_GPIO_Port GPIOA
#define VLV_CTRL_Pin GPIO_PIN_15
#define VLV_CTRL_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_10
#define SPI1_CS_GPIO_Port GPIOG
#define BUZZ_Pin GPIO_PIN_8
#define BUZZ_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_0
#define LED_BLUE_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
