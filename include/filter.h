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
 
 
 #define FILTER_LENGTH 4      // Number of samples used in filtering operations
 #define RPM_SCALE_FACTOR 1000  // Scale factor to increase resolution of filtered RPM values
 
 
 /* ------------------------------------------- Define structs -----------------------------------------*/

 /* Structure for Finite Impulse Response (FIR) filter implementation. 
 FIR filters use only input samples (no feedback) and have the form: y[n] = b0*x[n] + b1*x[n-1] + ... + bN*x[n-N] */
 typedef struct {
     float buffer[FILTER_LENGTH]; 
     uint8_t buffer_index;      
     float output;              
 } FIR_filter;
 

 /* Structure for Infinite Impulse Response (IIR) filter implementation
 IIR filters use both input and previous output samples and have the form:
 y[n] = b0*x[n] + b1*x[n-1] + ... + bN*x[n-N] - a1*y[n-1] - ... - aM*y[n-M] */
 typedef struct {
     float raw[FILTER_LENGTH+1];    
     float filtered[FILTER_LENGTH]; 
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
