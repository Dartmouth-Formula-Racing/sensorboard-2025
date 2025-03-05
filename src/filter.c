/*Author: Sasha Ries
 * Date: 1/25/25
 * File: filter.c
 * Description: functions to filter RPM values for front sensor board
 */

#include <math.h>
#include <stdint.h>
#include "filter.h"

/* --------------------------------------- Declare Global Variables and Local Functions -----------------------------------*/
static float FIR_impulse_response[WINDOW_SIZE] = {};

/* These values calculated in a seperate MATLAB script */

// Raw values are b values for IIR
static float IIR_raw_impulse[WINDOW_SIZE + 1] = {0.0898, 0.3594, 0.5391, 0.3594, 0.0898};
// Filtered values are a values for IIR
static float IIR_filtered_impulse[WINDOW_SIZE] = {-3.8358, 5.5208,-3.5335, 0.8486};


/* ----------------------------------------------------- IIR Filter -----------------------------------------------------*/
void IIR_filter_init(IIR_filter* filter){
    // Zero out all raw sample buffer values
    for (uint8_t n = 0; n < WINDOW_SIZE + 1; n++){
        filter->raw[n] = 0.0f;
    }

    // Zero out all filtered sample buffer values
    for (uint8_t n = 0; n < WINDOW_SIZE; n++){
        filter->filtered[n] = 0.0f;
    }

    filter->raw_index = 0;      // Reset raw buffer index
    filter->filtered_index = 0;  // Reset filtered buffer index
    filter->output = 0.0f;      // Clear filter output
}

float IIR_filter_update(IIR_filter* filter, float input){
    // Add new input to raw buffer at current index position
    filter->raw[filter->raw_index] = input;
    filter->raw_index++;
    
    // Reset index to 0 when it reaches end of buffer (circular buffer implementation)
    if (filter->raw_index == WINDOW_SIZE + 1){
        filter->raw_index = 0;
    }

    // Store previous output in filtered buffer
    filter->filtered[filter->filtered_index] = filter->output;
    filter->filtered_index++;
    
    // Reset index to 0 when it reaches end of buffer
    if (filter->raw_index == WINDOW_SIZE + 1){
        filter->raw_index = 0;
    }

    filter->output = 0.0f;  // Initialize filtered output sum
    
    // Process b coefficients (FIR portion) - traverse buffer in reverse chronological order
    uint8_t sumIndex = filter->raw_index;
    for (uint8_t n = 0; n < WINDOW_SIZE + 1; n++){ // Move backward through circular buffer
        if (sumIndex > 0){
            sumIndex--;
        }else{
            sumIndex = WINDOW_SIZE;  // Wrap around to end of buffer
        }
        // Standard FIR computation: b[n] * x[n-i]
        filter->output += IIR_raw_impulse[n] * filter->raw[sumIndex];
    }

    // Process a coefficients (IIR feedback portion) - traverse buffer in reverse order
    sumIndex = filter->filtered_index;
    for (uint8_t n = 0; n < WINDOW_SIZE; n++){ // Move backward through circular buffer
        if (sumIndex > 0){
            sumIndex--;
        }else{
            sumIndex = WINDOW_SIZE - 1;  // Wrap around to end of buffer
        }
        // Standard IIR computation: a[n] * y[n-i]
        filter->output += IIR_filtered_impulse[n] * filter->filtered[sumIndex];
    }
    
    return filter->output;
}


/* ------------------------------------------------ Rolling Average Filter -----------------------------------------------*/
int32_t Roll_average(sample_window* window, float new_speed) {
    window->data_array[window->position] = new_speed; // Store each value in new (rolling) index
    window->position = (window->position + 1) % WINDOW_SIZE; // Update position indicator

    // Average values in the sample window
    float sum = 0; 
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += window->data_array[i];
    }
    return (sum / (WINDOW_SIZE)) * RPM_SCALE_FACTOR; // Return the upscaled average
}



/* ----------------------------------------------------------- FIR Filter ---------------------------------------------- */
void FIR_filter_init(FIR_filter* filter){
    // Set all buffer values to zero
    for (uint8_t n = 0; n < WINDOW_SIZE; n++){
        filter->buffer[n] = 0.0f;
    }

    filter->buffer_index = 0;  // Initialize index to start of buffer
    filter->output = 0.0f;     // Clear output value
}

float FIR_filter_update(FIR_filter* filter, float input){
    // Store input at current buffer position
    filter->buffer[filter->buffer_index] = input;
    filter->buffer_index++;

    // Reset index to 0 when we reach end of buffer
    if (filter->buffer_index == WINDOW_SIZE){
        filter->buffer_index = 0;
    }

    filter->output = 0.0f;  // Reset accumulator for new calculation
    
    // Start from current index to access samples in reverse chronological order
    uint8_t sumIndex = filter->buffer_index;

    // Compute the convolution sum for FIR filter
    for (uint8_t n = 0; n < WINDOW_SIZE; n++){
        // Step backward through buffer (most recent to oldest samples)
        if (sumIndex > 0){
            sumIndex--;
        }else{
            sumIndex = WINDOW_SIZE - 1;  // Wrap to end when reaching beginning
        }

        // Implement FIR equation: y[n] = Σ h[i]·x[n-i]
        filter->output += FIR_impulse_response[n] * filter->buffer[sumIndex];
    }
    
    return filter->output;
}
