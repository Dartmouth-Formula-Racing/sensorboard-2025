/*Author: Sasha Ries
 * Date: 1/25/25
 * File: can.c
 * Description: defines all functions related to wheel speed calculation
 */

#include "stm32f0xx_hal_can.h"


// Functions to calculate wheel velocities
// float calculate_left_velocity(uint32_t delta_t);
// float calculate_right_velocity(uint32_t delta_t);

float calculate_left_velocity();
float calculate_right_velocity();

// External hardware interrupt function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);