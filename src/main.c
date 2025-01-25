/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal_can.h"
#include <filter.h>

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEND_INTERVAL 10 // milliseconds
#define COUNTS_PER_REVOLUTION 4096 // there are 2048 counts per revolution but we double it because channel A and B get double tiks
#define CAN_BASE_ADDRESS 0x65D // CAN address
#define CAN_USE_EXTENDED 0     // also idk might be yes
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

//Keep track of current tik counts
volatile int32_t A_count_left;
volatile int32_t A_count_right;
volatile int32_t B_count_left;
volatile int32_t B_count_right;
// volatile uint32_t Z_count_left;
// volatile uint32_t Z_count_right;

uint32_t last_send = 0;

//Keep track of previous tik counts 
int32_t last_A_left;
int32_t last_A_right;
int32_t last_B_left;
int32_t last_B_right;
// uint32_t last_Z_left;
// uint32_t last_Z_right;

sample_window window = {// Initialize sample window with all speeds = 0 and sample count = 0
    .data_array = {0},
    .sample_count = 0
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // Read current state of all pins (high or low) for quadrature
    uint8_t A_left_current = HAL_GPIO_ReadPin(A_left_tic_GPIO_Port, A_left_tic_Pin);
    uint8_t B_left_current = HAL_GPIO_ReadPin(B_left_tic_GPIO_Port, B_left_tic_Pin);
    uint8_t A_right_current = HAL_GPIO_ReadPin(A_right_tic_GPIO_Port, A_right_tic_Pin);
    uint8_t B_right_current = HAL_GPIO_ReadPin(B_right_tic_GPIO_Port, B_right_tic_Pin);

    // RIGHT SIDE: Counter-clockwise rotation has channel A leading channel B and is forward movement
    // LEFT SIDE: Clockwise rotation has channel A leading channel B and is forward movement


    // Edge trigger on A left wheel
    switch(GPIO_Pin){
      case A_left_tic_Pin:
        if(A_left_current == 1) {  // Rising edge on A
          if(B_left_current == 1) A_count_left--;  // Counter-clockwise
          else A_count_left++;                     // Clockwise
      } else {  // Falling edge on A
          if(B_left_current == 0) A_count_left--;  // Counter-clockwise
          else A_count_left++;                     // Clockwise
          }
        break;

      // Edge trigger on B left wheel
      case B_left_tic_Pin:
        if(B_left_current == 1) {  // Rising edge on B
          if(A_left_current == 0) B_count_left--;  // Counter-clockwise
          else B_count_left++;                     // Clockwise
      } else {  // Falling edge on B
          if(A_left_current == 1) B_count_left--;  // Counter-clockwise
          else B_count_left++;                     // Clockwise
          }
        break;

      // Edge trigger on A right wheel
      case A_right_tic_Pin:
        if(A_right_current == 1) {
          if(B_right_current == 1) A_count_right--;
          else A_count_right++;
      } else {
          if(B_right_current == 0) A_count_right--;
          else A_count_right++;
        }
        break;

      // Edge trigger on B right wheel
      case B_right_tic_Pin:
        if(B_right_current == 1) {
          if(A_right_current == 0) B_count_right--;
          else B_count_right++;
      } else {
          if(A_right_current == 1) B_count_right--;
          else B_count_right++;
        }
        break;

      // Z channel handling for higher speeds (just increments forward)
      // case Z_left_tic_Pin:
      //   Z_count_left++;
      //   break;

      // case Z_right_tic_Pin:
      //   Z_count_right++;
      //   break;
    }
}

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
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    uint32_t now = HAL_GetTick();

    if (now - last_send >= SEND_INTERVAL)
    {
      uint32_t delta_t = now - last_send; // most likely delta_t = SEND_INTERVAL but good to check

      // Calculate deltas and average in one step 
      // Use >> 1 to right shift by 1 bit for division by 2 (more efficient)
      volatile int32_t left_count_delta = ((A_count_left - last_A_left) + (B_count_left - last_B_left)) >> 1; 
      volatile int32_t right_count_delta = ((A_count_right - last_A_right) + (B_count_right - last_B_right)) >> 1;
      // volatile uint32_t Z_countLeft_delta = Z_count_left - last_Z_left;
      // volatile uint32_t Z_countRight_delta = Z_count_right - last_Z_right;

      // Calculate RPM values directly from averaged counts
      volatile float left_velocity = ((float)left_count_delta / COUNTS_PER_REVOLUTION) * (1000 * 60 / delta_t);
      volatile float right_velocity = ((float)right_count_delta / COUNTS_PER_REVOLUTION) * (1000 * 60 / delta_t);

      // Filter the velocities
#if FILTER_TYPE == 1
      left_velocity = Roll_average(&window, left_velocity);
      right_velocity = Roll_average(&window, right_velocity);
#else if FILTER_TYPE == 0
      // Dont filter the velocities, just scale them up
      left_velocity *= RPM_SCALE_FACTOR;
      right_velocity *= RPM_SCALE_FACTOR;
#endif

      // Calculate Z speeds by looking at difference in Z counts
      // volatile float left_speed_Z = ((float)Z_countLeft_delta) * (1000 * 60 / delta_t) / 2;// divide by 2 because z counts are double
      // volatile float right_speed_Z = ((float)Z_countRight_delta) * (1000 * 60 / delta_t) / 2;

      // Store current count values for next run through
      last_A_left = A_count_left;
      last_B_left = B_count_left;
      // last_Z_left = Z_count_left;
      last_A_right = A_count_right;
      last_B_right = B_count_right;
      // last_Z_right = Z_count_right;

      // Send values via CAN
      CAN_TxHeaderTypeDef tx_header;
      uint8_t data[8] = {0};
      tx_header.DLC = 8;
#if CAN_USE_EXTENDED
      tx_header.IDE = CAN_ID_EXT;
      tx_header.ExtId = CAN_BASE_ADDRESS;
#else
      tx_header.IDE = CAN_ID_STD;
      tx_header.StdId = CAN_BASE_ADDRESS;
#endif
      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
      {
        // Bytes 0 - 3 left speed
        data[0] = (((uint32_t)(left_velocity))) & 0xFF;
        data[1] = (((uint32_t)(left_velocity)) >> 8) & 0xFF;
        data[2] = (((uint32_t)(left_velocity)) >> 16) & 0xFF;
        data[3] = (((uint32_t)(left_velocity)) >> 24) & 0xFF;

        // Bytes 4 - 7 right speed
        data[4] = (((uint32_t)(right_velocity))) & 0xFF;
        data[5] = (((uint32_t)(right_velocity)) >> 8) & 0xFF;
        data[6] = (((uint32_t)(right_velocity)) >> 16) & 0xFF;
        data[7] = (((uint32_t)(right_velocity)) >> 24) & 0xFF;

        uint32_t tx_mailbox;
        HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox);
      }
      last_send = now;
    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  // Set up filter
  CAN_FilterTypeDef filter;
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  // Start CAN peripheral
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END CAN_Init 2 */
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
  GPIO_InitStruct.Pull = GPIO_PULLUP; /* no internal pull up or down resistors + voltage*/
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A_left_tic_Pin B_left_tic_Pin Z_left_tic_Pin */
  GPIO_InitStruct.Pin = A_left_tic_Pin | B_left_tic_Pin | Z_left_tic_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; /* set the pin to trigger on rising an falling edges*/
  GPIO_InitStruct.Pull = GPIO_PULLUP; /* no internal pull up or down resistors + voltage*/
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
