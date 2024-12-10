/**
  ******************************************************************************
  * @file    func.c
  * @brief   This file provides code for the polling of sensors and conversion of units.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "sensor.h"

#define VDD 3.3

/*----------------------------------------------------------------------------*/
/*  Polling Sensors                                                           */
/*----------------------------------------------------------------------------*/

// Polls the brake pressure switch
uint8_t pollBrake(void) {
  uint8_t brakeStatus = 0;
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) {
    brakeStatus = 1;
  }
  return brakeStatus;
}

// Polls the potentiometers
uint16_t pollPotentiometer(CAN_HandleTypeDef *hadc) {
  uint16_t adcValue = 0;
  HAL_ADC_Start(hadc);
  if (HAL_ADC_PollForConversion(hadc, 1000) == HAL_OK) {
    adcValue = HAL_ADC_GetValue(hadc);
  }
  HAL_ADC_Stop(hadc);
  return adcValue;
}

// Converts Encoder Count to RPM
uint16_t encoderToRpm(uint16_t encoderCount, uint16_t interval) {
  uint16_t rpm = 0;
  //Calculate RPM
  rpm = (encoderCount * 60 * 1000) / (ENCODER_COUNT_PER_REV * interval);
  return rpm;
}

/*----------------------------------------------------------------------------*/
/*  Utility Functions                                                         */
/*----------------------------------------------------------------------------*/

// Check if ADC Channel is on Steering Potentiometer when Interval Loop starts
void correctAdcChannel(uint8_t adcChannel, CAN_HandleTypeDef *hadc) {
  uint16_t temp = 0;
  if (adcChannel) { // adcChannel = 1 = Throttle Reading, we need Steering Reading
    HAL_ADC_Start(&hadc);
      if (HAL_ADC_PollForConversion(&hadc, 1000) == HAL_OK) {
          temp = HAL_ADC_GetValue(&hadc);  // Get the ADC value from Throttle Potentiometer
      }
      HAL_ADC_Stop(&hadc);
  }
}