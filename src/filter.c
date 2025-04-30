/*Author: Sasha Ries
 * Date: 3/3/25
 * File: filter.c
 * Description: functions to filter RPM values for Nucleo front sensor board
 */

 #include <math.h>
 #include <stdint.h>
 #include "filter.h"


 
 /* --------------------------------------- Declare Global Variables and Local Functions -----------------------------------*/
 static float FIR_impulse_response[FILTER_LENGTH] = {};
 
 /* These values calculated in a seperate MATLAB script */
 
//  // Raw values are b values for IIR
//  static float IIR_raw_impulse[FILTER_LENGTH + 1] = {0.0898, 0.3594, 0.5391, 0.3594, 0.0898};
//  // Filtered values are a values for IIR
//  static float IIR_filtered_impulse[FILTER_LENGTH] = {0.8358, -0.5208, 0.5335, 0.8486};

 // b coefficients (numerator)
static float IIR_raw_impulse[FILTER_LENGTH + 1] = {0.02785977, 0.05571953, 0.02785977};

// a coefficients (denominator, excluding a[0] which is normalized to 1)
static float IIR_filtered_impulse[FILTER_LENGTH] = {-1.47548044, 0.58691951};
 
 
 /* ----------------------------------------------------- IIR Filter -----------------------------------------------------*/
 void IIR_filter_init(IIR_filter* filter){
     // Zero out all raw sample buffer values
     for (uint8_t n = 0; n < FILTER_LENGTH + 1; n++){
         filter->raw[n] = 0.0f;
     }
 
     // Zero out all filtered sample buffer values
     for (uint8_t n = 0; n < FILTER_LENGTH; n++){
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
     if (filter->raw_index == FILTER_LENGTH + 1){
         filter->raw_index = 0;
     }
 
     // Store previous output in filtered buffer
     filter->filtered[filter->filtered_index] = filter->output;
     filter->filtered_index++;
     
     // Reset index to 0 when it reaches end of buffer
     if (filter->filtered_index == FILTER_LENGTH){
         filter->filtered_index = 0;
     }
 
     filter->output = 0.0f;  // Initialize filtered output sum
     
     // Process b coefficients (FIR portion) - traverse buffer in reverse chronological order
     uint8_t sumIndex = filter->raw_index;
     for (uint8_t n = 0; n < FILTER_LENGTH + 1; n++){ // Move backward through circular buffer
         if (sumIndex > 0){
             sumIndex--;
         }else{
             sumIndex = FILTER_LENGTH;  // Wrap around to end of buffer
         }
         // Standard FIR computation: b[n] * x[n-i]
         filter->output += IIR_raw_impulse[n] * filter->raw[sumIndex];
     }
 
     // Process a coefficients (IIR feedback portion) - traverse buffer in reverse order
     sumIndex = filter->filtered_index;
     for (uint8_t n = 0; n < FILTER_LENGTH; n++){ // Move backward through circular buffer
         if (sumIndex > 0){
             sumIndex--;
         }else{
             sumIndex = FILTER_LENGTH - 1;  // Wrap around to end of buffer
         }
         // Standard IIR computation: a[n] * y[n-i]
         filter->output -= IIR_filtered_impulse[n] * filter->filtered[sumIndex];
     }
     return filter->output;
 }
 
 
 /* ----------------------------------------------------------- FIR Filter ---------------------------------------------- */
 void FIR_filter_init(FIR_filter* filter){
     // Set all buffer values to zero
     for (uint8_t n = 0; n < FILTER_LENGTH; n++){
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
     if (filter->buffer_index == FILTER_LENGTH){
         filter->buffer_index = 0;
     }
 
     filter->output = 0.0f;  // Reset accumulator for new calculation
     
     // Start from current index to access samples in reverse chronological order
     uint8_t sumIndex = filter->buffer_index;
 
     // Compute the convolution sum for FIR filter
     for (uint8_t n = 0; n < FILTER_LENGTH; n++){
         // Step backward through buffer (most recent to oldest samples)
         if (sumIndex > 0){
             sumIndex--;
         }else{
             sumIndex = FILTER_LENGTH - 1;  // Wrap to end when reaching beginning
         }
 
         // Implement FIR equation: y[n] = Σ h[i]·x[n-i]
         filter->output += FIR_impulse_response[n] * filter->buffer[sumIndex];
     }
     
     return filter->output;
 }