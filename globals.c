//------------------------------------------------------------------------------
// globals.c  –  Global variable definitions
// All global variables used across multiple source files are defined here.
//------------------------------------------------------------------------------

#include "macros.h"

// Display
char display_line[4][11];
char *display[4];

// Timing (used by timers.c / main)
volatile unsigned int  Time_Sequence       = 0;
volatile char          one_time            = 1;

// Shape state (used by shapes.c, switches.c, init.c)
volatile unsigned char current_shape       = SHAPE_NONE;
volatile unsigned char shape_state         = 0;
volatile unsigned char shape_iteration     = 0;
volatile unsigned int  shape_start_time    = 0;

// Switch state flags (set in ISR, cleared in Switch*_Process)
volatile unsigned char sw1_pressed         = 0;
volatile unsigned char sw2_pressed         = 0;

// Shape selection (set by SW1, read by SW2)
unsigned char          selected_shape      = SHAPE_NONE;
