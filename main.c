/*------------------------------------------------------------------------------
 * File:        main.c.
 *
 *              Responsibilities assumed from timerB0.obj:
 *                - update_display is set TRUE every 200 ms by Timer B0 CCR0
 *                  ISR (in timerB0.c).  The foreground polls it here via
 *                  Display_Process() – no SPI calls occur inside any ISR.
 *                - Switch debounce is managed by CCR1 (SW1) and CCR2 (SW2)
 *                  in timerB0.c.
 *
 *              Main loop behavior:
 *                1. Calls Display_Process() every iteration.
 *                   - Display_Update() is only reached when both
 *                     update_display and display_changed are TRUE, which
 *                     guarantees >= 200 ms between LCD writes.
 *                2. Calls Switches_Process() every iteration.
 *                   - Detects sw1_pressed / sw2_pressed flags set by the
 *                     port ISRs and triggers a foreground display update.
 *
 *              Initial display content:
 *                Line 1: " Project  "
 *                Line 2: "  Timer   "
 *                Line 3: " Debounce "
 *                Line 4: "          "  (updated by switch presses)
 *
 * Globals used:
 *   display_line[4][11]  - LCD line buffers
 *   display[4]           - pointers to line buffers
 *   display_changed      - signals new content to Display_Process
 *   update_display       - set by CCR0 ISR; gated LCD refresh
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "timerB0.h"

/*------------------------------------------------------------------------------
 * Globals defined in globals.c – declare extern here
 *------------------------------------------------------------------------------*/
extern char                   display_line[4][11];
extern char                  *display[4];

/*------------------------------------------------------------------------------
 * Globals defined in timerB0.c – declare extern here
 *------------------------------------------------------------------------------*/
extern volatile unsigned char update_display;
extern volatile unsigned char display_changed;

/*==============================================================================
 * main
 *
 * Description: System entry point.
 *   1. Releases GPIO power-on high-impedance mode.
 *   2. Initializes all peripherals in order.
 *   3. Writes the startup message to the display buffers.
 *   4. Enters the infinite foreground service loop.
 *
 * Globals used:
 *   display_line[][]   - written with startup strings
 *   display_changed    - set TRUE to trigger first LCD refresh
 *   update_display     - polled by Display_Process (set by CCR0 ISR)
 *
 * Globals changed:
 *   display_line[0..3]
 *   display_changed
 *
 * Local variables:   none
 *============================================================================*/
void main(void){

    /* Release GPIO power-on default high-impedance mode */
    PM5CTL0 &= ~LOCKLPM5;

    /* Peripheral initialization order matters:
     *   Ports first (no floating pins during clock/timer init),
     *   then clocks, then conditions (enables GIE),
     *   then timers (CCR0 starts blinking immediately after GIE),
     *   then LCD (uses SPI which needs ports configured).           */
    Init_Ports();
    Init_Clocks();
    Init_Conditions();    /* also calls enable_interrupts() -> sets GIE        */
    Init_Timers();        /* TB0 CCR0 now running; backlight starts blinking   */
    Init_LCD();
    Init_Switches();      /* Final flag-clear and interrupt enable for SW1/SW2 */

    /* Load startup message into display line buffers */
    strncpy(display_line[0], " Project  ", DISPLAY_LINE_LENGTH - 1);
    strncpy(display_line[1], "  Timer   ", DISPLAY_LINE_LENGTH - 1);
    strncpy(display_line[2], " Debounce ", DISPLAY_LINE_LENGTH - 1);
    strncpy(display_line[3], "          ", DISPLAY_LINE_LENGTH - 1);

    /* Null-terminate each line for safety */
    display_line[0][DISPLAY_LINE_LENGTH - 1] = RESET_STATE;
    display_line[1][DISPLAY_LINE_LENGTH - 1] = RESET_STATE;
    display_line[2][DISPLAY_LINE_LENGTH - 1] = RESET_STATE;
    display_line[3][DISPLAY_LINE_LENGTH - 1] = RESET_STATE;

    /* Mark display as changed; CCR0 will set update_display within 200 ms */
    display_changed = TRUE;

    /*--------------------------------------------------------------------------
     * Foreground service loop
     *   Display_Process() – checks update_display (set by CCR0 every 200 ms)
     *                        and display_changed; calls Display_Update only
     *                        when both are true, keeping SPI writes out of ISRs.
     *   Switches_Process() – checks sw1_pressed / sw2_pressed flags from the
     *                         port ISRs and handles foreground display updates.
     *------------------------------------------------------------------------*/
    while(ALWAYS){
        Display_Process();
        Switches_Process();
    }
}
