//------------------------------------------------------------------------------
// Global Variables Definitions
// This file defines all global variables used across the project
//------------------------------------------------------------------------------

#include "macros.h"

// Display variables - display_line and display are defined here
// Note: display_changed, update_display, update_display_count are in LCD.c
char display_line[4][11];
char *display[4];

// Timing variables - used by timer.c
volatile unsigned int Time_Sequence = 0;
volatile char one_time = 1;

// Shape variables - used by shapes.c
volatile unsigned char current_shape = SHAPE_NONE;
volatile unsigned char shape_state = 0;
volatile unsigned char shape_iteration = 0;
volatile unsigned int shape_start_time = 0;

// Switch variables - used by switches.c
unsigned char selected_shape = SHAPE_NONE;
volatile unsigned char sw1_pressed = 0;
volatile unsigned char sw2_pressed = 0;
