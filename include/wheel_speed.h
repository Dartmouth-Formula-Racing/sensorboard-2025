/*Author: Sasha Ries
 * Date: 1/25/25
 * File: wheel_speed.h
 * Description: defines all functions related to wheel speed calculation
 */

#ifndef WHEEL_SPEED_H
#define WHEEL_SPEED_H

#include <stdint.h>
#define SEND_INTERVAL 10

/**
 * @brief Calculates the angular velocity of the left wheel
 * This function determines the velocity based on encoder count changes
 * over a specified time period.
 * 
 * @param delta_t Time elapsed since last calculation in milliseconds
 * @param count Current encoder count value for the left wheel
 * @return float Left wheel velocity in RPM (always positive)
 */
float calculate_left_velocity(uint32_t count);

/**
 * @brief Calculates the angular velocity of the right wheel
 * This function determines the velocity based on encoder count changes
 * over a specified time period.
 * 
 * @param delta_t Time elapsed since last calculation in milliseconds
 * @param count Current encoder count value for the right wheel
 * @return float Right wheel velocity in RPM (always positive)
 */
float calculate_right_velocity(uint32_t count);


#endif /* WHEEL_SPEED_H */