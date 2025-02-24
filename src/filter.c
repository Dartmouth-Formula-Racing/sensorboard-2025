/*Author: Sasha Ries
 * Date: 1/25/25
 * File: filter.c
 * Description: functions to filter RPM values for front sensor board
 */

#include <filter.h>
#include <math.h>

float biasArray[WINDOW_SIZE] = {2.0, 1.5, 1.0, 0.5};


// Function to sfilter data using rolling average
int32_t Roll_average(sample_window* window, float new_speed) {
    window->data_array[window->position] = new_speed;
    // Store each value in new (rolling) index
    window->position = (window->position + 1) % WINDOW_SIZE;

    // Average values in the sample window
    // float sum = 0; 
    // for (int i = 0; i < WINDOW_SIZE; i++) {
    //     sum += window->data_array[i];
    // }

    float sum = 0;
    int activePos;
    for (int i = 0; i < WINDOW_SIZE; i++){
        activePos = (window->position + i) % WINDOW_SIZE;
        sum += window->data_array[activePos] * biasArray[i];  
    }
    // Return the upscaled average
    return (sum / (WINDOW_SIZE * 5)) * RPM_SCALE_FACTOR;
}