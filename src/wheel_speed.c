/*Author: Sasha Ries
 * Date: 1/25/25
 * File: wheel_speed.c
 * Description: defines all functions related to wheel speed calculation
 */

#include "wheel_speed.h"
#include "main.h"

#define COUNTS_PER_REVOLUTION 2048  // Resolution of encoders

/*---------------------- Functions for wheel speed calculating ------------------------------------ */

float calculate_left_velocity(uint32_t count) {
    // Static variable to store previous count value between function calls
    static uint32_t last_left_count = 0;
    
    // if (last_left_count > count) {
    //     last_left_count = count; // Update stored count for next function call
    //     return left_velocity;
    // }

    // Calculate difference in encoder counts since last measurement
    int32_t count_delta = count - last_left_count;
    // Convert count delta to RPM:
    float left_velocity = ((float)count_delta / COUNTS_PER_REVOLUTION) * (1000.0 * 60.0 / SEND_INTERVAL);

    last_left_count = count; // Update stored count for next function call
    return left_velocity;
}


float calculate_right_velocity(uint32_t count) {
    // Static variable to store previous count value between function calls
    static uint32_t last_right_count = 0;
    
    // if (last_right_count > count) {
    //     last_right_count = count; // Update stored count for next function call
    //     return right_velocity;
    // }
    
    // Calculate difference in encoder counts since last measurement
    int32_t count_delta = count - last_right_count;
    // Convert count delta to RPM:
    float right_velocity = ((float)count_delta / COUNTS_PER_REVOLUTION) * (1000.0 * 60.0 / SEND_INTERVAL);

    last_right_count = count; // Update stored count for next function call
    return right_velocity;
}