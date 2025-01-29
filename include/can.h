/*Author: Sasha Ries
 * Date: 1/25/25
 * File: can.c
 * Description: defines all functions related to CAN communication
 */


// Function to send input data to predefined CAN addresses
void send_can1(float left_velocity, float right_velocity);
void send_can2(float left_velocity, float right_velocity);

// Function to initialize CAN communication
void MX_CAN_Init(void);