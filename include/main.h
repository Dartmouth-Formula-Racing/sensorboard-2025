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
#include "stm32f0xx_hal.h"

void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

// Original: #define Z_right_tic_Pin GPIO_PIN_7
#define Z_right_tic_Pin GPIO_PIN_7
#define Z_right_tic_GPIO_Port GPIOA
#define Z_right_tic_EXTI_IRQn EXTI4_15_IRQn

// Original: #define B_right_tic_Pin GPIO_PIN_6
#define B_right_tic_Pin GPIO_PIN_6
#define B_right_tic_GPIO_Port GPIOA 
#define B_right_tic_EXTI_IRQn EXTI4_15_IRQn

// Original: #define A_right_tic_Pin GPIO_PIN_5
#define A_right_tic_Pin GPIO_PIN_5
#define A_right_tic_GPIO_Port GPIOA
#define A_right_tic_EXTI_IRQn EXTI4_15_IRQn

// Original: #define Z_left_tic_Pin GPIO_PIN_15
#define Z_left_tic_Pin GPIO_PIN_15
#define Z_left_tic_GPIO_Port GPIOB
#define Z_left_tic_EXTI_IRQn EXTI4_15_IRQn

// Original: #define B_left_tic_Pin GPIO_PIN_14
#define B_left_tic_Pin GPIO_PIN_14
#define B_left_tic_GPIO_Port GPIOB
#define B_left_tic_EXTI_IRQn EXTI4_15_IRQn

// Original: #define A_left_tic_Pin GPIO_PIN_13
#define A_left_tic_Pin GPIO_PIN_13
#define A_left_tic_GPIO_Port GPIOB
#define A_left_tic_EXTI_IRQn EXTI4_15_IRQn





/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
