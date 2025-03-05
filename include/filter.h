/*
Author: Sasha Ries
Date: 1/25/25
File: filter.h
Description: Header file containing definitions for digital filters used to process RPM values
from the front sensor board. Includes FIR, IIR, and rolling average filter implementations.
 */

 #include <main.h>
 #include <stdbool.h>
 #include <stdint.h>
 
 
 #define WINDOW_SIZE 4      // Number of samples used in filtering operations
 #define FILTER_TYPE 1      // Selector value to determine which filter type to use in preprocessing
 #define RPM_SCALE_FACTOR 1000  // Scale factor to increase resolution of filtered RPM values
 #define FILTER_TYPE 1 // Value to pick which filter function gets used
 
 
 /* ------------------------------------------- Define structs -----------------------------------------*/
 /* Window struct to store a rolling window of samples and position indicator */
 typedef struct {
     float data_array[WINDOW_SIZE];
     uint32_t position;            
 } sample_window;
 

 /* Structure for Finite Impulse Response (FIR) filter implementation. 
 FIR filters use only input samples (no feedback) and have the form: y[n] = b0*x[n] + b1*x[n-1] + ... + bN*x[n-N] */
 typedef struct {
     float buffer[WINDOW_SIZE]; 
     uint8_t buffer_index;      
     float output;              
 } FIR_filter;
 

 /* Structure for Infinite Impulse Response (IIR) filter implementation
 IIR filters use both input and previous output samples and have the form:
 y[n] = b0*x[n] + b1*x[n-1] + ... + bN*x[n-N] - a1*y[n-1] - ... - aM*y[n-M] */
 typedef struct {
     float raw[WINDOW_SIZE+1];    
     float filtered[WINDOW_SIZE]; 
     uint8_t raw_index;           
     uint8_t filtered_index;      
     float output;                
 } IIR_filter;
 

 /**
  * Initializes an FIR filter structure
  * @param filter Pointer to the FIR_filter structure to initialize */
 void FIR_filter_init(FIR_filter* filter);
 

 /**
  * Updates an FIR filter with a new input sample and computes the new output
  * Implements the FIR filtering algorithm using predefined impulse response coefficients
  * @param filter Pointer to the FIR_filter structure to update
  * @param input New input sample to process
  * @return The newly computed filter output value
  */
 float FIR_filter_update(FIR_filter* filter, float input);
 

 /**
  * Initializes an IIR filter structure
  * @param filter Pointer to the IIR_filter structure to initialize
  */
 void IIR_filter_init(IIR_filter* filter);
 

 /**
  * Updates an IIR filter with a new input sample and computes the new output
  * Implements the IIR filtering algorithm using predefined raw (b) and filtered (a) impulse coefficients
  * @param filter Pointer to the IIR_filter structure to update
  * @param input New input sample to process
  * @return The newly computed filter output value
  */
 float IIR_filter_update(IIR_filter* filter, float input);
 
 /**
  * Implements a simple moving average (rolling average) filter 
  * @param window Pointer to the sample_window structure to update
  * @param new_speed New speed sample to add to the window
  * @return Scaled integer representation of the filtered RPM value (average * RPM_SCALE_FACTOR)
  */
 int32_t Roll_average(sample_window* window, float new_speed);