/*Author: Sasha Ries
 * Date: 1/25/25
 * File: main.c
 * Description: Runs main code for all functions related to front sensor board
 */

/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal_can.h"
#include "filter.h"
#include "can.h"
#include "wheel_speed.h"

#define SEND_INTERVAL 10           // milliseconds

/*----------------------------- Global structs -----------------------------*/

sample_window left_window = { // Initialize left sample window with all speeds = 0 and sample count = 0
    .data_array = {0},
    .position = 0};

sample_window right_window = { // Initialize right sample window with all speeds = 0 and sample count = 0
    .data_array = {0},
    .position = 0};

uint32_t last_send = 0; // Variable to keep track of time


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();

  /* Infinite loop */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if (now - last_send >= SEND_INTERVAL)
    {
      uint32_t delta_t = now - last_send; // Find time elapsed since last sample period

      // Calculate velocities of both wheels in RPM
      float left_velocity = calculate_left_velocity(delta_t);
      float right_velocity = calculate_right_velocity(delta_t);


      // Filter the velocities
#if FILTER_TYPE == 1
      float left_velocity_filtered = Roll_average(&left_window, left_velocity);
      float right_velocity_filtered = Roll_average(&right_window, right_velocity);
#else if FILTER_TYPE == 0
      // Dont filter the velocities, just scale them up
      left_velocity *= RPM_SCALE_FACTOR;
      right_velocity *= RPM_SCALE_FACTOR;
#endif

      // Scale it up (for testing purposes) ---- remove later
      left_velocity *= RPM_SCALE_FACTOR;
      right_velocity *= RPM_SCALE_FACTOR;

      // Send velocities via CAN
      send_can(left_velocity, right_velocity, left_velocity_filtered, right_velocity_filtered);

      last_send = now; // Update current time
    }
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : A_right_tic_Pin B_right_tic_Pin Z_right_tic_Pin */
  GPIO_InitStruct.Pin = A_right_tic_Pin | B_right_tic_Pin | Z_right_tic_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; /* set the pin to trigger on rising an falling edges*/
  GPIO_InitStruct.Pull = GPIO_PULLUP;                 /* no internal pull up or down resistors + voltage*/
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A_left_tic_Pin B_left_tic_Pin Z_left_tic_Pin */
  GPIO_InitStruct.Pin = A_left_tic_Pin | B_left_tic_Pin | Z_left_tic_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; /* set the pin to trigger on rising an falling edges*/
  GPIO_InitStruct.Pull = GPIO_PULLUP;                 /* no internal pull up or down resistors + voltage*/
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}


/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
