/*Author: Sasha Ries
 * Date: 1/25/25
 * File: can.c
 * Description: defines all functions related to CAN communication
 */


#include "can.h"
#include "main.h"
#include "stm32f0xx_hal_can.h"

#define CAN_BASE_ADDRESS 0x65D     // CAN base address
#define CAN_USE_EXTENDED 0         // also idk might be yes

CAN_HandleTypeDef hcan;

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */

/* ========================= Functions to send data via CAN =========================*/
void send_can1(float left_velocity, float right_velocity){

    CAN_TxHeaderTypeDef tx_header;
    tx_header.DLC = 8;
    tx_header.RTR = CAN_RTR_DATA;
#if CAN_USE_EXTENDED
    tx_header.IDE = CAN_ID_EXT;
    tx_header.ExtId = CAN_BASE_ADDRESS;
#else
    tx_header.IDE = CAN_ID_STD;
    tx_header.StdId = CAN_BASE_ADDRESS;
#endif

    uint8_t data[8] = {0};
    uint32_t tx_mailbox;

    // Bytes 0 - 3 left speed
    data[0] = ((uint32_t)(left_velocity)) & 0xFF;
    data[1] = (((uint32_t)(left_velocity)) >> 8) & 0xFF;
    data[2] = (((uint32_t)(left_velocity)) >> 16) & 0xFF;
    data[3] = (((uint32_t)(left_velocity)) >> 24) & 0xFF;

    // Bytes 4 - 7 right speed
    data[4] = ((uint32_t)(right_velocity)) & 0xFF;
    data[5] = (((uint32_t)(right_velocity)) >> 8) & 0xFF;
    data[6] = (((uint32_t)(right_velocity)) >> 16) & 0xFF;
    data[7] = (((uint32_t)(right_velocity)) >> 24) & 0xFF;

    // Actually send the bytes of data to CAN addresses
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0){
        if (HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox) != HAL_OK) {
            Error_Handler();
        }
    }
}

void send_can2(float left_velocity, float right_velocity){
    
    // Second CAN Address
    CAN_TxHeaderTypeDef tx_header2;
    tx_header2.DLC = 8;
    tx_header2.RTR = CAN_RTR_DATA;
#if CAN_USE_EXTENDED
    tx_header_filtered.IDE = CAN_ID_EXT;
    tx_header_filtered.ExtId = CAN_BASE_ADDRESS + 1;
#else
    tx_header2.IDE = CAN_ID_STD;
    tx_header2.StdId = CAN_BASE_ADDRESS + 1;
#endif

    uint8_t data[8] = {0};
    uint32_t tx_mailbox;

    // Bytes 0 - 3 left speed
    data[0] = ((uint32_t)(left_velocity)) & 0xFF;
    data[1] = (((uint32_t)(left_velocity)) >> 8) & 0xFF;
    data[2] = (((uint32_t)(left_velocity)) >> 16) & 0xFF;
    data[3] = (((uint32_t)(left_velocity)) >> 24) & 0xFF;

    // Bytes 4 - 7 right speed
    data[4] = ((uint32_t)(right_velocity)) & 0xFF;
    data[5] = (((uint32_t)(right_velocity)) >> 8) & 0xFF;
    data[6] = (((uint32_t)(right_velocity)) >> 16) & 0xFF;
    data[7] = (((uint32_t)(right_velocity)) >> 24) & 0xFF;

    // Actually send the bytes of data to CAN addresses
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0){
        if (HAL_CAN_AddTxMessage(&hcan, &tx_header2, data, &tx_mailbox) != HAL_OK) {
            Error_Handler();
        }
    }    

}

void MX_CAN_Init(void) {
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