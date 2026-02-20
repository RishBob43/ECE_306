//------------------------------------------------------------------------------
// Global Variables Definitions
// This file defines all global variables used across the project
//------------------------------------------------------------------------------
#include "macros.h"

// Display variables
char display_line[4][11];
char *display[4];

// SMCLK control variables
volatile unsigned char smclk_output_enabled = TRUE;   // Start with SMCLK output on
volatile unsigned char smclk_freq_500k = FALSE;       // Start at 8MHz

// Switch variables
volatile unsigned char sw1_pressed = 0;
volatile unsigned char sw2_pressed = 0;
