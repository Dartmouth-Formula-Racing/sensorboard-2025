/*
 * Author: Sasha Ries
 * Date: 1/25/25
 * File: can.h
 * Description: Header file that defines the interface for CAN communication
 *              including initialization and data transmission functions
 */

 #include <main.h>

 /**
  * Sends wheel velocity data over the CAN bus
  * @param left_velocity  The left wheel velocity value to transmit (RPM)
  * @param right_velocity The right wheel velocity value to transmit (RPM)
  * @param offset         Offset value added to the base CAN ID to determine final message ID,
  *                       allows sending to different addresses with the same function
  */
 void send_can(float left_velocity, float right_velocity, uint16_t offset);
 
 /**
  * Initializes the CAN peripheral with appropriate settings
  * This function:
  * - Configures CAN timing parameters for proper baud rate
  * - Sets up the message acceptance filter
  * - Enables automatic features (bus-off recovery, auto wake-up, retransmission)
  * - Starts the CAN peripheral after configuration
  * Must be called once at system startup before any CAN communication
  */
 void MX_CAN_Init(void);