//------------------------------------------------------------------------------
//
//  Description: Main Routine - SMCLK Demonstration
//  SW1: Toggle P3.4 between GPIO and SMCLK output
//  SW2: Toggle SMCLK frequency between 8MHz and 500kHz
//
//  Built with Code Composer Version: CCS12.4.0.00007_win64
//------------------------------------------------------------------------------

#include  "msp430.h"
#include  <string.h>
#include  "functions.h"
#include  "LCD.h"
#include  "ports.h"
#include  "macros.h"

// Function Prototypes
void main(void);
void Init_Conditions(void);
void Display_Process(void);

// Global Variables
extern char display_line[4][11];
extern char *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char update_display;

// SMCLK control variables
extern volatile unsigned char smclk_output_enabled;  // Start with SMCLK output enabled
extern volatile unsigned char smclk_freq_500k;      // Start at 8MHz

void main(void){
//------------------------------------------------------------------------------
// Main Program - SMCLK Demonstration
//------------------------------------------------------------------------------
  PM5CTL0 &= ~LOCKLPM5;
  // Disable the GPIO power-on default high-impedance mode

  Init_Clocks();                       // Initialize Clock System (8MHz)
  Init_Ports();                        // Initialize Ports
  Init_Conditions();                   // Initialize Variables
  Init_Timers();                       // Initialize Timers
  Init_LCD();                          // Initialize LCD
  Init_Switches();                     // Initialize Switches

  // Initial display
  strcpy(display_line[0], "P3.4:SMCLK");
  strcpy(display_line[1], "FREQ:8MHz ");
  strcpy(display_line[2], "SW1:P3.4  ");
  strcpy(display_line[3], "SW2:FREQ  ");
  display_changed = TRUE;

//------------------------------------------------------------------------------
// Main Loop
//------------------------------------------------------------------------------
  while(ALWAYS) {
    Switches_Process();                // Process switch inputs
    Display_Process();                 // Update Display
  }
//------------------------------------------------------------------------------
}
