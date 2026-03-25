/*------------------------------------------------------------------------------
 * File:        globals.c  (Project 7)
 * Target:      MSP430FR2355
 *
 * Description: Project-wide global variable definitions.
 *
 * update_display_count:
 *   Queued tick counter incremented by the CCR0 ISR every 200 ms.
 *   The foreground main loop decrements it by 1 each pass so that if the
 *   loop is slow (LCD write, ADC read) it can "catch up" on missed ticks
 *   rather than silently dropping them.
 *
 * update_display:
 *   Legacy boolean flag kept for compatibility with Display_Process() and
 *   other LCD driver functions that still read it.  The main loop sets this
 *   TRUE each time it consumes a count from update_display_count.
 *------------------------------------------------------------------------------*/

#include "macros.h"

/*-- Display buffers ----------------------------------------------------------*/
char  display_line[4][11];
char *display[4];

/*-- Timer / state machine ----------------------------------------------------*/
volatile unsigned int  Time_Sequence = RESET_STATE;
volatile char          one_time      = RESET_STATE;

/*-- Display tick counter (queued; drains one per main-loop pass) ------------*/
extern volatile unsigned int  update_display_count;

/*-- Legacy boolean display flag (set by foreground when count consumed) -----*/
extern volatile unsigned char update_display;

/*-- Switch state flags -------------------------------------------------------*/
volatile unsigned char sw1_pressed = FALSE;
volatile unsigned char sw2_pressed = FALSE;
