

#include "macros.h"

// Display buffers
char display_line[4][11];
char *display[4];

// Timer / state machine
volatile unsigned int  Time_Sequence = 0;
volatile char          one_time      = 0;

// Switch state flags
volatile unsigned char sw1_pressed   = 0;
volatile unsigned char sw2_pressed   = 0;
