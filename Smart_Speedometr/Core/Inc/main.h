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
#include "stm32g0xx_hal.h"

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
#define BAT_Pin GPIO_PIN_3
#define BAT_GPIO_Port GPIOA
#define HALL_Pin GPIO_PIN_5
#define HALL_GPIO_Port GPIOA
#define Sw3_Pin GPIO_PIN_6
#define Sw3_GPIO_Port GPIOA
#define Sw3_EXTI_IRQn EXTI4_15_IRQn
#define Sw2_Pin GPIO_PIN_7
#define Sw2_GPIO_Port GPIOA
#define Sw2_EXTI_IRQn EXTI4_15_IRQn
#define Sw1_Pin GPIO_PIN_0
#define Sw1_GPIO_Port GPIOB
#define Sw1_EXTI_IRQn EXTI0_1_IRQn
#define STDBY_BAT_Pin GPIO_PIN_11
#define STDBY_BAT_GPIO_Port GPIOA
#define STDBY_BAT_EXTI_IRQn EXTI4_15_IRQn
#define CHRG_BAT_Pin GPIO_PIN_12
#define CHRG_BAT_GPIO_Port GPIOA
#define CHRG_BAT_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
