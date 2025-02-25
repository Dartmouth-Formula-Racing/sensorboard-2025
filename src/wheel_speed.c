/*Author: Sasha Ries
 * Date: 1/25/25
 * File: can.c
 * Description: defines all functions related to wheel speed calculation
 */

#include "main.h"
#include "wheel_speed.h"

#define COUNTS_PER_REVOLUTION 2048 // there are 2048 counts per revolution but we double it because channel A and B get double tiks

/*------------------------------------- Global constants for wheel speed calculating ---------------------------------------*/

volatile uint32_t prev_time_left = 0;
volatile uint32_t cur_time_left = 0;
volatile uint32_t prev_time_right = 0;
volatile uint32_t cur_time_right = 0;



/*---------------------------------------- Functions for wheel speed calculating ------------------------------------*/
float calculate_left_velocity(){
  // Calculate time interval
  uint32_t left_time_delta = cur_time_left - prev_time_left;

  if (left_time_delta == 0){ // Add check to prevent division by 0
    left_time_delta += 5;
  }

  // Calculate RPM values directly from time interval
  float left_velocity = (60*1000)/((float)left_time_delta*COUNTS_PER_REVOLUTION);

  return left_velocity; // return left wheel velocity in RPM
}


float calculate_right_velocity(){
  // Calculate time interval
  uint32_t right_time_delta = cur_time_right - prev_time_right;

  if (right_time_delta == 0){ // Add check to prevent division by 0
    right_time_delta += 5;
  }

  // Calculate RPM values directly from time interval
  float right_velocity = (60*1000)/((float)right_time_delta*COUNTS_PER_REVOLUTION);

  return right_velocity; // return right wheel velocity in RPM
}



/* ---------------------------- External hardware interrupt to store timer counts in software ----------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  switch (GPIO_Pin){
    case A_left_tic_Pin:
      prev_time_left = cur_time_left;
      cur_time_left = HAL_GetTick();
      break;
    
    case A_right_tic_Pin:
      prev_time_right = cur_time_right;
      cur_time_right = HAL_GetTick();
      break;
  }
}

