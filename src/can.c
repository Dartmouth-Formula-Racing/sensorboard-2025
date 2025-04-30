/*Author: Sasha Ries
 * Date: 1/25/25
 * File: can.c
 * Description: defines all functions related to CAN communication
 */


 #include "main.h"
 #include "stm32f0xx_hal.h"
 #include "can.h"

#define CAN_BASE_ADDRESS 0x65D     // CAN base address
#define CAN_USE_EXTENDED 0         // also idk might be yes

// CAN_HandleTypeDef hcan;

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */

/* ========================= Functions to send data via CAN =========================*/

void send_can(CAN_HandleTypeDef *can, float left_velocity, float right_velocity, uint16_t offset){
    CAN_TxHeaderTypeDef tx_header;
    tx_header.DLC = 8;
    tx_header.RTR = CAN_RTR_DATA;
#if CAN_USE_EXTENDED
    tx_header_filtered.IDE = CAN_ID_EXT;
    tx_header_filtered.ExtId = CAN_BASE_ADDRESS + offset;
#else
    tx_header.IDE = CAN_ID_STD;
    tx_header.StdId = CAN_BASE_ADDRESS + offset;
#endif

    uint8_t data[8] = {0};
    uint32_t tx_mailbox;

    // Bytes 0 - 3 left speed
    data[0] = (((uint32_t)(left_velocity)) >> 24) & 0xFF;
    data[1] = (((uint32_t)(left_velocity)) >> 16) & 0xFF;
    data[2] = (((uint32_t)(left_velocity)) >> 8) & 0xFF;
    data[3] = ((uint32_t)(left_velocity)) & 0xFF;

    // Bytes 4 - 7 right speed
    data[4] = (((uint32_t)(right_velocity)) >> 24) & 0xFF;
    data[5] = (((uint32_t)(right_velocity)) >> 16) & 0xFF;
    data[6] = (((uint32_t)(right_velocity)) >> 8) & 0xFF;
    data[7] = ((uint32_t)(right_velocity)) & 0xFF;

    // Actually send the bytes of data to CAN addresses
    if (HAL_CAN_GetTxMailboxesFreeLevel(can) > 0){
        if (HAL_CAN_AddTxMessage(can, &tx_header, data, &tx_mailbox) != HAL_OK) {
            Error_Handler();
        }
    }    

}