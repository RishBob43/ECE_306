/*------------------------------------------------------------------------------
 * File:        main.c  (Project 6)
 * Target:      MSP430FR2355
 *
 * Description: TA demo sequence.
 *              Motors are driven full ON or full OFF via GPIO - no PWM.
 *              Display updates every 200 ms when content has changed.
 *
 *   1. SW1 pressed -> 1 second delay
 *   2. Car drives forward (motors full on)
 *   3. Either detector sees black line -> stop, display "LINE DET  "
 *   4. Wait 4 seconds
 *   5. Turn left (right motor on, left motor off) until BOTH detectors
 *      are over the black line
 *   6. Stop, display "ON LINE   " and live detector value on line 3
 *------------------------------------------------------------------------------*/

#include <ADC.h>
#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "timerB0.h"
#include "hex_to_bcd.h"

/*------------------------------------------------------------------------------
 * Externs
 *------------------------------------------------------------------------------*/
extern char                   display_line[4][11];
extern char                  *display[4];
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char update_display;
extern volatile unsigned char display_changed;

/*------------------------------------------------------------------------------
 * State machine states
 *------------------------------------------------------------------------------*/
#define STATE_IDLE          (0)
#define STATE_DELAY         (1)
#define STATE_FORWARD       (2)
#define STATE_STOP_WAIT     (3)
#define STATE_TURN          (4)
#define STATE_DONE          (5)

/*------------------------------------------------------------------------------
 * Timing  (1 tick = 200 ms from CCR0)
 *------------------------------------------------------------------------------*/
#define TICKS_1_SEC         (5u)
#define TICKS_4_SEC         (20u)

/*------------------------------------------------------------------------------
 * Motor control macros
 *   R_FORWARD and L_FORWARD are P6 output pins connected directly to the
 *   motor driver.  Writing the pin HIGH drives the motor full on.
 *   R_REVERSE / L_REVERSE pins are kept LOW at all times here.
 *------------------------------------------------------------------------------*/
#define MOTORS_FORWARD()    do { P6OUT |=  (R_FORWARD | L_FORWARD); \
                                 P6OUT &= ~(R_REVERSE | L_REVERSE); } while(0)

#define MOTORS_STOP()       do { P6OUT &= ~(R_FORWARD | L_FORWARD | \
                                             R_REVERSE | L_REVERSE); } while(0)

#define MOTOR_TURN_BACK_RIGHT()   do { P6OUT |=  R_REVERSE;               \
                                 P6OUT &= ~(L_FORWARD | R_FORWARD | \
                                             L_REVERSE); } while(0)

/*------------------------------------------------------------------------------
 * Helper: write a 10-char string into a display line
 *------------------------------------------------------------------------------*/
static void set_line(int line, const char *msg){
    strncpy(display_line[line], msg, DISPLAY_LINE_LENGTH - 1);
    display_line[line][DISPLAY_LINE_LENGTH - 1] = RESET_STATE;
    display_changed = TRUE;
}

/*------------------------------------------------------------------------------
 * Helper: write live left detector BCD value to line 3
 *   Format: "L:xxxx    "
 *------------------------------------------------------------------------------*/
static void show_detector_value(void){
    HEX_to_BCD(ADC_Right_Detect);
    display_line[1][0] = 'R';
    display_line[1][1] = ':';
    display_line[1][2] = thousands;
    display_line[1][3] = hundreds;
    display_line[1][4] = tens;
    display_line[1][5] = ones;
    display_line[1][6] = ' ';
    display_line[1][7] = ' ';
    display_line[1][8] = ' ';
    display_line[1][9] = ' ';
    display_line[1][DISPLAY_LINE_LENGTH - 1] = RESET_STATE;
    HEX_to_BCD(ADC_Left_Detect);
    display_line[3][0] = 'L';
    display_line[3][1] = ':';
    display_line[3][2] = thousands;
    display_line[3][3] = hundreds;
    display_line[3][4] = tens;
    display_line[3][5] = ones;
    display_line[3][6] = ' ';
    display_line[3][7] = ' ';
    display_line[3][8] = ' ';
    display_line[3][9] = ' ';
    display_line[3][DISPLAY_LINE_LENGTH - 1] = RESET_STATE;
    display_changed = TRUE;
}

/*==============================================================================
 * main
 *==============================================================================*/
void main(void){

    unsigned char state       = STATE_IDLE;
    unsigned int  state_timer = RESET_STATE;

    PM5CTL0 &= ~LOCKLPM5;

    Init_Ports();
    Init_Clocks();
    Init_Conditions();
    Init_Timers();
    Init_LCD();
    Init_ADC();
    Init_Switches();

    /* IR emitter on from the start */
    IR_LED_control(IR_LED_ON);

    /* Make sure all motors start off */
    MOTORS_STOP();

    /* Startup display */
    set_line(0, "IR  Detect");
    set_line(1, "Thumb:0000");
    set_line(2, "          ");
    set_line(3, "Press SW1 ");

    /*==========================================================================
     * Main loop
     *========================================================================*/
    while(ALWAYS){

        /*----------------------------------------------------------------------
         * Service the display every 200 ms when content has changed.
         * Display_Process() only calls Display_Update() when BOTH
         * update_display AND display_changed are TRUE.
         *--------------------------------------------------------------------*/
        Display_Process();

        /*----------------------------------------------------------------------
         * Update thumbwheel on line 1 every 200 ms tick
         *--------------------------------------------------------------------*/
        if(update_display){
            Update_Thumb_Display();
        }

        /*----------------------------------------------------------------------
         * State machine
         *--------------------------------------------------------------------*/
        switch(state){

            /*------------------------------------------------------------------
             * IDLE – wait for SW1
             *----------------------------------------------------------------*/
            case STATE_IDLE:
                if(sw1_pressed){
                    sw1_pressed = FALSE;
                    state_timer = RESET_STATE;
                    state       = STATE_DELAY;
                    set_line(3, "DELAY...  ");
                }
                break;

            /*------------------------------------------------------------------
             * DELAY – 1 second pause before moving
             *----------------------------------------------------------------*/
            case STATE_DELAY:
                if(update_display){
                    state_timer++;
                    if(state_timer >= TICKS_1_SEC){
                        state_timer = RESET_STATE;
                        state       = STATE_FORWARD;
                        MOTORS_FORWARD();
                        set_line(3, "FORWARD   ");
                    }
                }
                break;

            /*------------------------------------------------------------------
             * FORWARD – drive until either detector sees the black line
             *----------------------------------------------------------------*/
            case STATE_FORWARD:
                if(Get_Line_State() != LINE_NONE){
                    MOTORS_STOP();
                    state_timer = RESET_STATE;
                    state       = STATE_STOP_WAIT;
                    set_line(2, "   LINE   ");
                    set_line(3, " DETECTED ");
                }
                break;

            /*------------------------------------------------------------------
             * STOP_WAIT – hold still for 4 seconds
             *----------------------------------------------------------------*/
            case STATE_STOP_WAIT:
                if(update_display){
                    state_timer++;
                    if(state_timer >= TICKS_4_SEC){
                        state_timer = RESET_STATE;
                        state       = STATE_TURN;
                        MOTOR_TURN_BACK_RIGHT();
                        set_line(2, "TURNING   ");
                    }
                }
                break;

            /*------------------------------------------------------------------
             * TURN – right motor on, left motor off.
             *        Stop when both detectors see the black line.
             *        Safety timeout after 8 seconds.
             *----------------------------------------------------------------*/
            case STATE_TURN:
                if(Get_Line_State() == LINE_BOTH){
                    MOTORS_STOP();
                    state       = STATE_DONE;
                    set_line(2, "ON LINE   ");
                    show_detector_value();
                } else if(update_display){
                    state_timer++;
                    if(state_timer >= TICKS_4_SEC * 2u){
                        MOTORS_STOP();
                        state = STATE_DONE;
                        set_line(2, "ON LINE   ");
                        show_detector_value();
                    }
                }
                break;

            /*------------------------------------------------------------------
             * DONE – display live detector value every 200 ms.
             *        SW1 resets back to idle for another run.
             *----------------------------------------------------------------*/
            case STATE_DONE:
                if(update_display){
                    show_detector_value();
                }
                if(sw1_pressed){
                    sw1_pressed = FALSE;
                    state       = STATE_IDLE;
                    set_line(2, "          ");
                    set_line(3, "Press SW1 ");
                }
                break;

            default:
                state = STATE_IDLE;
                break;
        }

    } /* while(ALWAYS) */
}
