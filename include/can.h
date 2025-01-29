/*Author: Sasha Ries
 * Date: 1/25/25
 * File: can.c
 * Description: defines all functions related to CAN communication
 */


// Function to send input data to predefined CAN addresses
void send_can(float left_velocity, float right_velocity, float left_velocity_filtered, float right_velocity_filtered);

// Function to initialize CAN communication
void MX_CAN_Init(void);