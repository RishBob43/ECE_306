//------------------------------------------------------------------------------
// init.c  –  System Initialization
//------------------------------------------------------------------------------

#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

extern char display_line[4][11];
extern char *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char update_display;
extern volatile unsigned int  update_display_count;
extern volatile unsigned int  Time_Sequence;
extern volatile char          one_time;
extern volatile unsigned char current_shape;
extern volatile unsigned char shape_state;
extern volatile unsigned char shape_iteration;
extern volatile unsigned int  shape_start_time;
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;
extern unsigned char          selected_shape;

void Init_Conditions(void){
    int i;

    // Clear all display buffers
    for(i = 0; i < 11; i++){
        display_line[0][i] = RESET_STATE;
        display_line[1][i] = RESET_STATE;
        display_line[2][i] = RESET_STATE;
        display_line[3][i] = RESET_STATE;
    }
    // Null-terminate each line
    display_line[0][10] = 0;
    display_line[1][10] = 0;
    display_line[2][10] = 0;
    display_line[3][10] = 0;

    // Point display pointers at the line buffers
    display[0] = &display_line[0][0];
    display[1] = &display_line[1][0];
    display[2] = &display_line[2][0];
    display[3] = &display_line[3][0];

    update_display = 0;

    // Shape variables
    current_shape    = SHAPE_NONE;
    shape_state      = 0;
    shape_iteration  = 0;
    shape_start_time = 0;
    selected_shape   = SHAPE_NONE;

    // Switch flags
    sw1_pressed = 0;
    sw2_pressed = 0;

    // Enable global interrupts
    enable_interrupts();
}
