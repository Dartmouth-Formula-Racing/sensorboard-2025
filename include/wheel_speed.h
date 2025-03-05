/*Author: Sasha Ries
 * Date: 1/25/25
 * File: wheel_speed.h
 * Description: defines all functions related to wheel speed calculation
 */

 #include "stm32f0xx_hal_can.h"

 /* -------------------------------------- Function Declarations ---------------------------------------- */
 

 /**
  * Calculates the left wheel velocity in RPM based on encoder counts
  * @param delta_t Time period in milliseconds between velocity calculations
  * @return Left wheel velocity in RPM, positive values indicate forward rotation
  */
 float calculate_left_velocity(uint32_t delta_t);
 


 /**
  * Calculates the right wheel velocity in RPM based on encoder counts
  * @param delta_t Time period in milliseconds between velocity calculations
  * @return Right wheel velocity in RPM, positive values indicate forward rotation
  */
 float calculate_right_velocity(uint32_t delta_t);
 


 /**
  * Interrupt handler for quadrature encoder inputs 
  * For left wheel: Clockwise rotation = forward movement
  * For right wheel: Counter-clockwise rotation = forward movement
  * @param GPIO_Pin The pin that triggered the interrupt (A_left, B_left, A_right, or B_right)
  */
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);