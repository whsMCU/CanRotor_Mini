/*
 * LED_control.c
 *
 *  Created on: 2019. 1. 26.
 *      Author: WANG
 */

//#include "LED_control.h"
#include "Board.h"

uint8_t flight_mode, flight_mode_counter, flight_mode_led, headfree_mode_counter, headfree_mode_led;
uint32_t flight_mode_timer, headfree_mode_timer;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the error LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void error_signal(void) {
  if (Error.error >= 100) {RGB_R_ON;}                                                         //When the error is 100 the LED is always on.
  else if (Error.error_timer < millis()) {                                                       //If the error_timer value is smaller that the millis() function.
    Error.error_timer = millis() + 250;                                                          //Set the next error_timer interval at 250ms.
    if (Error.error > 0 && Error.error_counter > Error.error + 3) Error.error_counter = 0;                         //If there is an error to report and the error_counter > error +3 reset the error.
    if (Error.error_counter < Error.error && Error.error_led == 0 && Error.error > 0) {                            //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
      RGB_R_ON;                                                                       //Turn the LED on.
      Error.error_led = 1;                                                                       //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
      RGB_R_OFF;                                                                        //Turn the LED off.
      Error.error_counter++;                                                                     //Increment the error_counter variable by 1 to keep trach of the flashes.
      Error.error_led = 0;                                                                       //Set the LED flag to indicate that the LED is off.
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the flight mode LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void flight_mode_signal(void) {
  if (flight_mode_timer < millis()) {                                                      //If the error_timer value is smaller that the millis() function.
    flight_mode_timer = millis() + 250;                                                    //Set the next error_timer interval at 250ms.
    if (f.ARMED > 0 && flight_mode_counter > flight_mode + 3) flight_mode_counter = 0; //If there is an error to report and the error_counter > error +3 reset the error.
    if (flight_mode_counter < f.ARMED && flight_mode_led == 0 && f.ARMED > 0) {    //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
      RGB_G_ON;                                                                     //Turn the LED on.
      flight_mode_led = 1;                                                                 //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
      RGB_G_OFF;                                                                      //Turn the LED off.
      flight_mode_counter++;                                                               //Increment the error_counter variable by 1 to keep trach of the flashes.
      flight_mode_led = 0;                                                                 //Set the LED flag to indicate that the LED is off.
    }
  }

  if (headfree_mode_timer < millis()) {                                                      //If the error_timer value is smaller that the millis() function.
    headfree_mode_timer = millis() + 250;                                                    //Set the next error_timer interval at 250ms.
    if (f.HEADFREE_MODE > 0 && headfree_mode_counter > flight_mode + 3) headfree_mode_counter = 0; //If there is an error to report and the error_counter > error +3 reset the error.
    if (headfree_mode_counter < f.HEADFREE_MODE && headfree_mode_led == 0 && f.HEADFREE_MODE > 0) {    //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
      RGB_B_ON;                                                                     //Turn the LED on.
      headfree_mode_led = 1;                                                                 //Set the LED flag to indicate that the LED is on.
    }
    else {                                                                                 //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
      RGB_B_OFF;                                                                      //Turn the LED off.
      headfree_mode_counter++;                                                               //Increment the error_counter variable by 1 to keep trach of the flashes.
      headfree_mode_led = 0;                                                                 //Set the LED flag to indicate that the LED is off.
    }
  }
}

