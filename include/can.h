/*Author: Sasha Ries
 * Date: 1/25/25
 * File: can.c
 * Description: defines all functions related to CAN communication
 */
#include <main.h>

// Function to send input data to predefined CAN addresses
void send_can(float left_velocity, float right_velocity, uint16_t offset);

// Function to initialize CAN communication
void MX_CAN_Init(void);