/*Author: Sasha Ries
 * Date: 1/25/25
 * File: can.c
 * Description: defines all functions related to CAN communication
 */


#include "can.h"
#include "main.h"
#include "stm32f0xx_hal_can.h"

#define CAN_BASE_ADDRESS 0x65D     // CAN base address
#define CAN_USE_EXTENDED 0

CAN_HandleTypeDef hcan; // Declare hcan struct


/* ========================= Functions to send data via CAN =========================*/
void send_can(float left_velocity, float right_velocity, uint16_t offset){
    
  // Setup CAN message header structure
  CAN_TxHeaderTypeDef tx_header;
  tx_header.DLC = 8;                // Data Length Code: 8 bytes
  tx_header.RTR = CAN_RTR_DATA;     // Regular data frame, not a remote frame
#if CAN_USE_EXTENDED
  tx_header_filtered.IDE = CAN_ID_EXT;
  tx_header_filtered.ExtId = CAN_BASE_ADDRESS + offset;
#else
  tx_header.IDE = CAN_ID_STD;       // Using standard CAN ID format
  tx_header.StdId = CAN_BASE_ADDRESS + offset;  // Set message ID with offset
#endif
  
  uint8_t data[8] = {0};            // Initialize data buffer with zeros
  uint32_t tx_mailbox;
  
  // Bytes 0 - 3 left speed: Convert float to bytes using bitshifting
  data[0] = (((uint32_t)(left_velocity)) >> 24) & 0xFF;
  data[1] = (((uint32_t)(left_velocity)) >> 16) & 0xFF;
  data[2] = (((uint32_t)(left_velocity)) >> 8) & 0xFF;
  data[3] = ((uint32_t)(left_velocity)) & 0xFF;
  
  // Bytes 4 - 7 right speed: Convert float to bytes using bitshifting
  data[4] = (((uint32_t)(right_velocity)) >> 24) & 0xFF;
  data[5] = (((uint32_t)(right_velocity)) >> 16) & 0xFF;
  data[6] = (((uint32_t)(right_velocity)) >> 8) & 0xFF;
  data[7] = ((uint32_t)(right_velocity)) & 0xFF;
  
  // Only attempt to send if a mailbox is available
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0){
      // Add message to transmission queue
      if (HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox) != HAL_OK) {
          Error_Handler();  // Call error handler if transmission setup fails
      }
  }
  
}

void MX_CAN_Init(void) {
// Initialize CAN hardware configuration
hcan.Instance = CAN;
hcan.Init.Prescaler = 6;           // Sets the baud rate
hcan.Init.Mode = CAN_MODE_NORMAL;  // Standard operation mode 
hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
hcan.Init.TimeTriggeredMode = DISABLE;
hcan.Init.AutoBusOff = ENABLE;     // Auto recovery from Bus-Off state
hcan.Init.AutoWakeUp = ENABLE;     // Auto wake up from sleep mode
hcan.Init.AutoRetransmission = ENABLE;  // Retry transmission if failed
hcan.Init.ReceiveFifoLocked = DISABLE;
hcan.Init.TransmitFifoPriority = DISABLE;
if (HAL_CAN_Init(&hcan) != HAL_OK)
{
  Error_Handler();  // Call error handler if initialization fails
}

// Set up acceptance filter - currently set to accept all messages
CAN_FilterTypeDef filter;
filter.FilterBank = 0;
filter.FilterMode = CAN_FILTERMODE_IDMASK;
filter.FilterScale = CAN_FILTERSCALE_32BIT;
filter.FilterIdHigh = 0x0000;        // ID bits 16-31
filter.FilterIdLow = 0x0000;         // ID bits 0-15
filter.FilterMaskIdHigh = 0x0000;    // Mask bits 16-31 (0 = don't care)
filter.FilterMaskIdLow = 0x0000;     // Mask bits 0-15 (0 = don't care)
filter.FilterFIFOAssignment = CAN_RX_FIFO0;  // Route matches to FIFO0
filter.FilterActivation = ENABLE;
filter.SlaveStartFilterBank = 14;
if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
{
  Error_Handler();  // Call error handler if filter setup fails
}

// Start CAN peripheral after configuration is complete
if (HAL_CAN_Start(&hcan) != HAL_OK)
{
  Error_Handler();  // Call error handler if CAN won't start
}
}