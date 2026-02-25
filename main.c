//------------------------------------------------------------------------------
// main.c  –  Main Routine / "While" Operating System
//
// SW1: toggle P3.4 between SMCLK output (500 kHz) and GPIO input
// SW2: (available for future use)
//------------------------------------------------------------------------------

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

void main(void);

// Externals from globals.c
extern char display_line[4][11];
extern char *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char update_display;
extern volatile unsigned int  update_display_count;
extern volatile unsigned int  Time_Sequence;
extern volatile char          one_time;

//------------------------------------------------------------------------------
void main(void){
    // Disable GPIO power-on high-impedance mode
    PM5CTL0 &= ~LOCKLPM5;

    Init_Ports();          // Configure all GPIO / peripheral pins
    Init_Clocks();         // MCLK = 8 MHz, SMCLK = 500 kHz (DIVS=/16)
    Init_Conditions();     // Zero variables, enable interrupts
    Init_Timers();         // Configure Timer_B peripherals
    Init_LCD();            // Bring up SPI LCD
    Init_Switches();       // Configure SW1 / SW2 interrupts

    // Initial display: show P3.4 starts as GPIO
    strcpy(display_line[0], "P3.4= GPIO");
    strcpy(display_line[1], "          ");
    strcpy(display_line[2], "SW1:Toggle");
    strcpy(display_line[3], " CLK/GPIO ");
    display_changed = TRUE;

    //--------------------------------------------------------------------------
    // "While" Operating System
    //--------------------------------------------------------------------------
    while(ALWAYS){
        Switches_Process();   // SW1 / SW2
        Display_Process();    // Refresh LCD if display_changed
        Carlson_StateMachine();
        P3OUT ^= TEST_PROBE;  // Toggle P3.0 test probe each loop pass
    }
}
