//------------------------------------------------------------------------------
// main.c  –  Main Routine / "While" Operating System
//
// Shape Drawing with SMCLK / GPIO P3.4 toggle via switches
//   SW1: cycle shape selection  AND  configure P3.4 as SMCLK output (500 kHz)
//   SW2: start selected shape   AND  configure P3.4 as GPIO input
//
// See switches.c for full behaviour description.
//------------------------------------------------------------------------------

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

// Prototypes for functions defined in this file
void main(void);

// Globals owned by this file
volatile char slow_input_down;
unsigned char display_mode;
unsigned int  test_value;
char          chosen_direction;
char          change;
unsigned int  wheel_move;
char          forward;

// Externals from globals.c
extern char display_line[4][11];
extern char *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char update_display;
extern volatile unsigned int  update_display_count;
extern volatile unsigned int  Time_Sequence;
extern volatile char          one_time;
extern volatile unsigned char current_shape;
extern unsigned char          selected_shape;

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

    // Initial display: shape-select menu, P3.4 starts as GPIO
    strcpy(display_line[0], "SELECT:   ");
    strcpy(display_line[1], "  IDLE    ");
    strcpy(display_line[2], " SW1:Next ");
    strcpy(display_line[3], "P3.4= GPIO");
    display_changed = TRUE;

    wheel_move     = 0;
    forward        = TRUE;
    current_shape  = SHAPE_NONE;
    selected_shape = SHAPE_NONE;

    //--------------------------------------------------------------------------
    // "While" Operating System
    //--------------------------------------------------------------------------
    while(ALWAYS){
        Switches_Process();   // SW1 / SW2 (must be first)
        //Shapes_Process();     // Active shape state machines
        Display_Process();    // Refresh LCD if display_changed
        P3OUT ^= TEST_PROBE;  // Toggle P3.0 test probe each loop pass
    }
}
