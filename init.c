/*------------------------------------------------------------------------------
 * File:        init.c  (Project 8)
 * Target:      MSP430FR2355
 *
 * Description: System initialization – clears display buffers, sets pointers,
 *              clears all flags, enables global interrupts.
 *------------------------------------------------------------------------------*/

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
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;

void Init_Conditions(void){
    int i;

    /* Clear all four 11-byte display line buffers */
    for(i = 0; i < 11; i++){
        display_line[0][i] = RESET_STATE;
        display_line[1][i] = RESET_STATE;
        display_line[2][i] = RESET_STATE;
        display_line[3][i] = RESET_STATE;
    }

    /* Point display[] pointers at the line buffers */
    display[0] = &display_line[0][0];
    display[1] = &display_line[1][0];
    display[2] = &display_line[2][0];
    display[3] = &display_line[3][0];

    /* Clear all flags and counters */
    update_display       = FALSE;
    update_display_count = RESET_STATE;
    display_changed      = FALSE;
    sw1_pressed          = FALSE;
    sw2_pressed          = FALSE;
    Time_Sequence        = RESET_STATE;
    one_time             = RESET_STATE;

    /* Enable global interrupts */
    enable_interrupts();
}
