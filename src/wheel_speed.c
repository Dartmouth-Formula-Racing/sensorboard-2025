/*Author: Sasha Ries
 * Date: 1/25/25
 * File: wheel_speed.c
 * Description: defines all functions related to wheel speed calculation
 */

#include "main.h"
#include "wheel_speed.h"

#define COUNTS_PER_REVOLUTION 4096 // there are 2048 counts per revolution but we double it because channel A and B get double tiks

/*---------------------------------------------- Define Global Constants ---------------------------------------------*/
// Keep track of current tik counts
volatile int32_t A_count_left;
volatile int32_t A_count_right;
volatile int32_t B_count_left;
volatile int32_t B_count_right;

// Keep track of previous tik counts
int32_t last_A_left;
int32_t last_A_right;
int32_t last_B_left;
int32_t last_B_right;


/*------------------------------------- Functions for wheel speed calculating ------------------------------------*/
float calculate_left_velocity(uint32_t delta_t){
    // Calculate deltas and average in one step
    int32_t left_count_delta = ((A_count_left - last_A_left) + (B_count_left - last_B_left)) >> 2;

    // Calculate RPM values directly from averaged counts
    float left_velocity = ((float)left_count_delta / COUNTS_PER_REVOLUTION) * (1000 * 60 / delta_t);

    // Store current count values for next run through
    last_A_left = A_count_left;
    last_B_left = B_count_left;

    return left_velocity; // return left wheel velocity in RPM
}

float calculate_right_velocity(uint32_t delta_t){
    // Calculate deltas and average in one step
    int32_t right_count_delta = ((A_count_right - last_A_right) + (B_count_right - last_B_right)) >> 2;

    // Calculate RPM values directly from averaged counts
    float right_velocity = ((float)right_count_delta / COUNTS_PER_REVOLUTION) * (1000.0 * 60.0 / delta_t);

    // Store current count values for next run through
    last_A_right = A_count_right;
    last_B_right = B_count_right;

    return right_velocity; // return right wheel velocity in RPM
}


/* ------------------------- External hardware interrupt to count encoder tics in software ---------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  // Read current state of all pins (high or low) for quadrature
  uint8_t A_left_current = HAL_GPIO_ReadPin(A_left_tic_GPIO_Port, A_left_tic_Pin);
  uint8_t B_left_current = HAL_GPIO_ReadPin(B_left_tic_GPIO_Port, B_left_tic_Pin);
  uint8_t A_right_current = HAL_GPIO_ReadPin(A_right_tic_GPIO_Port, A_right_tic_Pin);
  uint8_t B_right_current = HAL_GPIO_ReadPin(B_right_tic_GPIO_Port, B_right_tic_Pin);

  // RIGHT SIDE: Counter-clockwise rotation has channel A leading channel B and is forward movement
  // LEFT SIDE: Clockwise rotation has channel A leading channel B and is forward movement

  switch (GPIO_Pin){
  case A_left_tic_Pin: // Edge trigger on A left wheel
    if (A_left_current == 1)
    { // Rising edge on A
      if (B_left_current == 1)
        A_count_left--; // Counter-clockwise
      else
        A_count_left++; // Clockwise
    }
    else
    { // Falling edge on A
      if (B_left_current == 0)
        A_count_left--; // Counter-clockwise
      else
        A_count_left++; // Clockwise
    }
    break;

  // Edge trigger on B left wheel
  case B_left_tic_Pin:
    if (B_left_current == 1)
    { // Rising edge on B
      if (A_left_current == 0)
        B_count_left--; // Counter-clockwise
      else
        B_count_left++; // Clockwise
    }
    else
    { // Falling edge on B
      if (A_left_current == 1)
        B_count_left--; // Counter-clockwise
      else
        B_count_left++; // Clockwise
    }
    break;

  // Edge trigger on A right wheel
  case A_right_tic_Pin:
    if (A_right_current == 1)
    {
      if (B_right_current == 1)
        A_count_right--;
      else
        A_count_right++;
    }
    else
    {
      if (B_right_current == 0)
        A_count_right--;
      else
        A_count_right++;
    }
    break;

  // Edge trigger on B right wheel
  case B_right_tic_Pin:
    if (B_right_current == 1)
    {
      if (A_right_current == 0)
        B_count_right--;
      else
        B_count_right++;
    }
    else
    {
      if (A_right_current == 1)
        B_count_right--;
      else
        B_count_right++;
    }
    break;
  }
}