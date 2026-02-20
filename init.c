#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

extern char display_line[4][11];
extern char *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char update_display;

void Init_Conditions(void){
//------------------------------------------------------------------------------
// Initialize Variables and Initial Conditions for SMCLK Demo
//------------------------------------------------------------------------------

  int i;

  // Initialize display buffers
  for(i=0; i<11; i++){
    display_line[0][i] = RESET_STATE;
    display_line[1][i] = RESET_STATE;
    display_line[2][i] = RESET_STATE;
    display_line[3][i] = RESET_STATE;
  }

  // Null terminate strings
  display_line[0][10] = 0;
  display_line[1][10] = 0;
  display_line[2][10] = 0;
  display_line[3][10] = 0;

  // Set display pointers
  display[0] = &display_line[0][0];
  display[1] = &display_line[1][0];
  display[2] = &display_line[2][0];
  display[3] = &display_line[3][0];

  update_display = 0;

  // Enable interrupts
  enable_interrupts();
//------------------------------------------------------------------------------
}
