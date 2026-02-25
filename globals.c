//------------------------------------------------------------------------------
// globals.c  –  Global variable definitions
// All global variables used across multiple source files are defined here.
//------------------------------------------------------------------------------

#include "macros.h"

// Display
char display_line[4][11];
char *display[4];



// Switch state flags (set in ISR, cleared in Switch*_Process)
volatile unsigned char sw1_pressed         = 0;
volatile unsigned char sw2_pressed         = 0;
