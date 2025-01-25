/*Author: Sasha Ries
 * Date: 1/25/25
 * File: filter.c
 * Description: functions to filter RPM values for front sensor board
 */

#include <filter.h>
#include <math.h>

// Function to set up a rolling average filter
int32_t Roll_average(sample_window* window, float new_speed){
    window-> data_array[window->sample_count] = new_speed;
    window->sample_count = (window->sample_count + 1) % WINDOW_SIZE; // Move position indicator on window index

    // Average values in the sample window
    int sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += window->data_array[i];
    }
    // Increase precision of by multiplying and dividing by 1000
    return (int32_t)(RPM_SCALE_FACTOR * (sum / WINDOW_SIZE));
}