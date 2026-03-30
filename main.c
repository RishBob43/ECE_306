/*------------------------------------------------------------------------------
 * File:        main.c  (Homework 8)
 * Target:      MSP430FR2355
 *
 * Description: UCA0/UCA1 serial communication with cross-connect ISRs,
 *              baud switching via SW1/SW2, and LCD display.
 *
 *              Startup:
 *                Splash screen for 5 s:
 *                  Line 0: "ECE306    "
 *                  Line 1: "NCSU  HW8 "
 *                  Line 2: "   Baud   "
 *                  Line 3: " 115,200  "
 *
 *              SW1  ->  115,200 baud
 *                  Clear lines 0 & 1, update line 3 to "115,200"
 *                  Wait 2 s, transmit "NCSU  #1" out UCA1
 *                  Line 0 shows last 10 chars received from UCA1
 *
 *              SW2  ->  460,800 baud  (same sequence, different rate)
 *
 *              LCD assignment during active operation:
 *                Line 0:  last 10 chars received  (rolling)
 *                Line 1:  "          "  (cleared on baud change)
 *                Line 2:  "   Baud   "
 *                Line 3:  " 115,200  " or " 460,800  "
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "timerB0.h"
#include "serial.h"

/*==============================================================================
 * Externs
 *==============================================================================*/
extern char                   display_line[4][11];
extern char                  *display[4];
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;
extern volatile unsigned char update_display;
extern volatile unsigned int  update_display_count;
extern volatile unsigned char display_changed;

/* From serial.c */
extern volatile char          rx_display_buf[RX_DISPLAY_LEN + 1u];
extern volatile unsigned char rx_display_updated;
extern volatile unsigned char current_baud;

/*==============================================================================
 * State definitions
 *==============================================================================*/
#define STATE_SPLASH        (0u)
#define STATE_IDLE          (1u)
#define STATE_BAUD_WAIT     (2u)   /* 2 s pause after baud change + transmit  */
#define STATE_ACTIVE        (3u)   /* show received chars, await next SW press */

/*==============================================================================
 * Baud rate display strings (10 chars, space-padded for centering)
 *==============================================================================*/
static const char baud_str_115200[] = " 115,200  ";
static const char baud_str_460800[] = " 460,800  ";

/*==============================================================================
 * Helpers
 *==============================================================================*/

static void set_line(int line, const char *msg){
    strncpy(display_line[line], msg, DISPLAY_LINE_LENGTH - 1u);
    display_line[line][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;
    display_changed = TRUE;
}

static void show_baud_line(void){
    if(current_baud == BAUD_460800){
        set_line(3, baud_str_460800);
    } else {
        set_line(3, baud_str_115200);
    }
}

/*------------------------------------------------------------------------------
 * refresh_rx_line0
 *
 * Copies the 10-char rolling buffer rx_display_buf (maintained in the ISR)
 * directly into display_line[0].  The buffer already contains exactly
 * RX_DISPLAY_LEN (10) printable characters so no further formatting is needed.
 * Non-printable characters are replaced with a '.' to keep the LCD clean.
 *------------------------------------------------------------------------------*/
static void refresh_rx_line0(void){
    unsigned int i;
    char c;
    for(i = 0; i < RX_DISPLAY_LEN; i++){
        c = rx_display_buf[i];
        /* Replace non-printable (control chars, DEL, high-bit) with '.'      */
        if(c < 0x20 || c > 0x7E){
            c = '.';
        }
        display_line[0][i] = c;
    }
    display_line[0][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;
    display_changed = TRUE;
}

/*==============================================================================
 * do_baud_change
 *
 * Common code for SW1 and SW2: reconfigure ports, update LCD lines 0-3.
 *==============================================================================*/
static void do_baud_change(unsigned char baud){
    Set_Baud_UCA0(baud);
    Set_Baud_UCA1(baud);   /* also clears rx_display_buf to spaces            */

    set_line(0, "          ");
    set_line(1, "          ");
    set_line(2, "   Baud   ");
    show_baud_line();
}

/*==============================================================================
 * main
 *==============================================================================*/
void main(void){

    unsigned char state       = STATE_SPLASH;
    unsigned int  state_timer = RESET_STATE;
    unsigned char tick_200ms  = FALSE;

    PM5CTL0 &= ~LOCKLPM5;

    Init_Ports();
    Init_Clocks();
    Init_Conditions();
    Init_Timers();
    Init_LCD();

    /* Initialize both serial ports at 115,200 baud */
    Init_Serial_UCA0(BAUD_115200);
    Init_Serial_UCA1(BAUD_115200);

    /* Show splash immediately before entering the loop */
    set_line(0, "ECE306    ");
    set_line(1, "NCSU  HW8 ");
    set_line(2, "   Baud   ");
    set_line(3, baud_str_115200);
    Display_Update(0, 0, 0, 0);

    /*==========================================================================
     * Main loop
     *========================================================================*/
    while(ALWAYS){

        /*----------------------------------------------------------------------
         * Consume one 200 ms tick
         *--------------------------------------------------------------------*/
        tick_200ms = FALSE;
        if(update_display_count > 0u){
            update_display_count--;
            update_display = TRUE;
            tick_200ms     = TRUE;
        }

        /*----------------------------------------------------------------------
         * If ISR flagged a new received character, copy to LCD line 0
         *--------------------------------------------------------------------*/
        if(rx_display_updated && (state == STATE_ACTIVE)){
            rx_display_updated = FALSE;
            refresh_rx_line0();
        }

        /*----------------------------------------------------------------------
         * Refresh LCD
         *--------------------------------------------------------------------*/
        Display_Process();

        /*----------------------------------------------------------------------
         * State machine
         *--------------------------------------------------------------------*/
        switch(state){

            /*------------------------------------------------------------------
             * SPLASH – hold for 5 s (25 ticks × 200 ms)
             *----------------------------------------------------------------*/
            case STATE_SPLASH:
                if(tick_200ms){
                    state_timer++;
                }
                if(state_timer >= SPLASH_TICKS){
                    state_timer = RESET_STATE;
                    /* Discard any accidental presses during splash */
                    sw1_pressed = FALSE;
                    sw2_pressed = FALSE;
                    /* Transition: clear top lines, keep baud lines */
                    set_line(0, "          ");
                    set_line(1, "          ");
                    state = STATE_IDLE;
                }
                break;

            /*------------------------------------------------------------------
             * IDLE – display current baud, wait for SW press
             *----------------------------------------------------------------*/
            case STATE_IDLE:
                if(sw1_pressed){
                    sw1_pressed = FALSE;
                    do_baud_change(BAUD_115200);
                    state_timer = RESET_STATE;
                    state = STATE_BAUD_WAIT;
                }
                if(sw2_pressed){
                    sw2_pressed = FALSE;
                    do_baud_change(BAUD_460800);
                    state_timer = RESET_STATE;
                    state = STATE_BAUD_WAIT;
                }
                break;

            /*------------------------------------------------------------------
             * BAUD_WAIT – 2 s delay (10 ticks), then transmit NCSU #1
             *----------------------------------------------------------------*/
            case STATE_BAUD_WAIT:
                if(tick_200ms){
                    state_timer++;
                }
                if(state_timer >= POST_BAUD_CHANGE_TICKS){
                    state_timer = RESET_STATE;
                    /* Send NCSU #1 out UCA1 to PC */
                    UCA1_Transmit_String(NCSU_STRING, NCSU_STRING_LEN);
                    state = STATE_ACTIVE;
                }
                break;

            /*------------------------------------------------------------------
             * ACTIVE – display received chars on line 0, await next SW press
             *----------------------------------------------------------------*/
            case STATE_ACTIVE:
                if(sw1_pressed){
                    sw1_pressed = FALSE;
                    do_baud_change(BAUD_115200);
                    state_timer = RESET_STATE;
                    state = STATE_BAUD_WAIT;
                }
                if(sw2_pressed){
                    sw2_pressed = FALSE;
                    do_baud_change(BAUD_460800);
                    state_timer = RESET_STATE;
                    state = STATE_BAUD_WAIT;
                }
                break;

            default:
                state = STATE_IDLE;
                break;
        }

    } /* while(ALWAYS) */
}
