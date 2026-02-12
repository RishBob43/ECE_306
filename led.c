#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "msp430.h"
extern char display_line[4][11];
extern char *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char update_display;
extern volatile unsigned int update_display_count;
extern volatile unsigned int Time_Sequence;
extern volatile char one_time;

void Init_LEDs(void){
//------------------------------------------------------------------------------
// LED Configurations
//------------------------------------------------------------------------------
// Turns on both LEDs
  P1OUT &= ~RED_LED;
  P6OUT &= ~GRN_LED;
//------------------------------------------------------------------------------
}

void Carlson_StateMachine(void){
    switch(Time_Sequence){
      case 250:                        //
        if(one_time){
          Init_LEDs();
          lcd_BIG_mid();
          display_changed = 1;
          one_time = 0;
        }
        Time_Sequence = 0;             //
        break;
      case 200:                        //
        if(one_time){
//          P1OUT &= ~RED_LED;            // Change State of LED 4
          P6OUT |= GRN_LED;            // Change State of LED 5
          one_time = 0;
        }
        break;
      case 150:                         //
        if(one_time){
          P1OUT |= RED_LED;            // Change State of LED 4
          P6OUT &= ~GRN_LED;           // Change State of LED 5

          // Motor Control - Start moving forward to travel 5 inches
          P6OUT |= R_FORWARD;          // Turn on right motor forward direction
          P6OUT |= L_FORWARD;          // Turn on left motor forward direction
          P6OUT &= ~R_REVERSE;         // Ensure right motor reverse is off
          P6OUT &= ~L_REVERSE;         // Ensure left motor reverse is off

          one_time = 0;
        }
        break;
      case 100:                         //
        if(one_time){
//          lcd_4line();
          lcd_BIG_bot();
          P6OUT |= GRN_LED;            // Change State of LED 5
          display_changed = 1;
          one_time = 0;
        }
        break;
      case  50:                        //
        if(one_time){
          // Motor Control - Stop all motors after ~5 inch movement
          // Motors run from count 150 to 50 = 100 time units
          P6OUT &= ~R_FORWARD;         // Turn off right motor forward direction
          P6OUT &= ~L_FORWARD;         // Turn off left motor forward direction
          P6OUT &= ~R_REVERSE;         // Ensure right motor reverse remains off
          P6OUT &= ~L_REVERSE;         // Ensure left motor reverse remains off

          one_time = 0;
        }
        break;                         //
      default: break;
    }
}
