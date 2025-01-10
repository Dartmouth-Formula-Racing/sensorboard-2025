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
#define Z_right_tic_Pin GPIO_PIN_5    // Now receives Z_right signal
#define Z_right_tic_GPIO_Port GPIOA //swapped
#define Z_right_tic_EXTI_IRQn EXTI4_15_IRQn

// Original: #define B_right_tic_Pin GPIO_PIN_6
#define B_right_tic_Pin GPIO_PIN_6    // Now recieves A_right signal
#define B_right_tic_GPIO_Port GPIOA //swapped
#define B_right_tic_EXTI_IRQn EXTI4_15_IRQn

// Original: #define Z_right_tic_Pin GPIO_PIN_7
#define A_right_tic_Pin GPIO_PIN_7    // Now receives B_right signal
#define A_right_tic_GPIO_Port GPIOA
#define A_right_tic_EXTI_IRQn EXTI4_15_IRQn

// Original: #define A_left_tic_Pin GPIO_PIN_13
#define A_left_tic_Pin GPIO_PIN_13    // This stays the same
#define A_left_tic_GPIO_Port GPIOB
#define A_left_tic_EXTI_IRQn EXTI4_15_IRQn

// Original: #define B_left_tic_Pin GPIO_PIN_14
#define Z_left_tic_Pin GPIO_PIN_14    // Now receives B_left signal
#define Z_left_tic_GPIO_Port GPIOB
#define Z_left_tic_EXTI_IRQn EXTI4_15_IRQn

// Original: #define Z_left_tic_Pin GPIO_PIN_15
#define B_left_tic_Pin GPIO_PIN_15    // Now receives Z_left signal
#define B_left_tic_GPIO_Port GPIOB
#define B_left_tic_EXTI_IRQn EXTI4_15_IRQn




/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
