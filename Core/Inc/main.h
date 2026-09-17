/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define PC13_Pin GPIO_PIN_13
#define PC13_GPIO_Port GPIOC
#define PC14_Pin GPIO_PIN_14
#define PC14_GPIO_Port GPIOC
#define PC15_Pin GPIO_PIN_15
#define PC15_GPIO_Port GPIOC
#define Power_ADC_Pin GPIO_PIN_0
#define Power_ADC_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_0
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_2
#define LED_B_GPIO_Port GPIOB
#define M4_B_Pin GPIO_PIN_10
#define M4_B_GPIO_Port GPIOB
#define M4_A_Pin GPIO_PIN_11
#define M4_A_GPIO_Port GPIOB
#define Limit_SW_1_Pin GPIO_PIN_13
#define Limit_SW_1_GPIO_Port GPIOB
#define Limit_SW_2_Pin GPIO_PIN_14
#define Limit_SW_2_GPIO_Port GPIOB
#define M3_B_Pin GPIO_PIN_6
#define M3_B_GPIO_Port GPIOC
#define M3_A_Pin GPIO_PIN_7
#define M3_A_GPIO_Port GPIOC
#define M2_B_Pin GPIO_PIN_8
#define M2_B_GPIO_Port GPIOC
#define M2_A_Pin GPIO_PIN_9
#define M2_A_GPIO_Port GPIOC
#define M1_B_Pin GPIO_PIN_8
#define M1_B_GPIO_Port GPIOA
#define M1_A_Pin GPIO_PIN_9
#define M1_A_GPIO_Port GPIOA
#define M5_A_Pin GPIO_PIN_15
#define M5_A_GPIO_Port GPIOA
#define PC12_Pin GPIO_PIN_12
#define PC12_GPIO_Port GPIOC
#define M5_B_Pin GPIO_PIN_3
#define M5_B_GPIO_Port GPIOB
#define M6_A_Pin GPIO_PIN_6
#define M6_A_GPIO_Port GPIOB
#define M6_B_Pin GPIO_PIN_7
#define M6_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
