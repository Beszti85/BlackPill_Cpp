/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ESP_AT_REPORT_EVENT_FLAG_MASK 0x00000001uL
#define USB_EVENT_FLAG_MASK 0x00000004uL
#define ESP_AT_RESPONSE_EVENT_FLAG_MASK 0x00000002uL
#define LED_BRD_Pin GPIO_PIN_13
#define LED_BRD_GPIO_Port GPIOC
#define OUT_PC14_Pin GPIO_PIN_14
#define OUT_PC14_GPIO_Port GPIOC
#define CE_RF_Pin GPIO_PIN_15
#define CE_RF_GPIO_Port GPIOC
#define CS_FLASH_Pin GPIO_PIN_4
#define CS_FLASH_GPIO_Port GPIOA
#define CS_RF_Pin GPIO_PIN_2
#define CS_RF_GPIO_Port GPIOB
#define CS_SD_Pin GPIO_PIN_10
#define CS_SD_GPIO_Port GPIOB
#define OUT_PB12_Pin GPIO_PIN_12
#define OUT_PB12_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
