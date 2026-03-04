/*------------------------------------------------------------------------------
 * File:        switches.c
 * Target:      MSP430FR2355
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "timerB0.h"
#include <string.h>

extern volatile char         debounce_flags;
extern volatile unsigned int sw1_debounce_count;
extern volatile unsigned int sw2_debounce_count;

extern char                  display_line[4][11];
extern char                  *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char update_display;

extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;

/*------------------------------------------------------------------------------
 * PORT4_VECTOR  –  SW1
 *------------------------------------------------------------------------------*/
#pragma vector = PORT4_VECTOR
__interrupt void switch1_interrupt(void){

    if(P4IFG & SW1){
        if(!(P4IN & SW1)){

            P4IE  &= ~SW1;

            debounce_flags |= SW1_DEBOUNCE_ACTIVE_BIT;
            sw1_debounce_count = RESET_STATE;

            TB0CCR1  = TB0R + TB0CCR1_DEBOUNCE_SLICE;
            TB0CCTL1 |= CCR_INTERRUPT_ENABLE;

            sw1_pressed = TRUE;
        }
        P4IFG &= ~SW1;
    }
}

/*------------------------------------------------------------------------------
 * PORT2_VECTOR  –  SW2
 *------------------------------------------------------------------------------*/
#pragma vector = PORT2_VECTOR
__interrupt void switch2_interrupt(void){

    if(P2IFG & SW2){
        if(!(P2IN & SW2)){

            P2IE  &= ~SW2;

            debounce_flags |= SW2_DEBOUNCE_ACTIVE_BIT;
            sw2_debounce_count = RESET_STATE;

            TB0CCR2  = TB0R + TB0CCR2_DEBOUNCE_SLICE;
            TB0CCTL2 |= CCR_INTERRUPT_ENABLE;

            sw2_pressed = TRUE;
        }
        P2IFG &= ~SW2;
    }
}

/*------------------------------------------------------------------------------
 * Init_Switches
 *------------------------------------------------------------------------------*/
void Init_Switches(void){
    P4IFG &= ~SW1;
    P4IE  |=  SW1;
    P2IFG &= ~SW2;
    P2IE  |=  SW2;
}

/*------------------------------------------------------------------------------
 * Switch1_Process
 *------------------------------------------------------------------------------*/
void Switch1_Process(void){
    if(sw1_pressed){
        sw1_pressed    = FALSE;
        update_display = TRUE;
        Display_Process();
    }
}

/*------------------------------------------------------------------------------
 * Switch2_Process
 *------------------------------------------------------------------------------*/
void Switch2_Process(void){
    if(sw2_pressed){
        sw2_pressed    = FALSE;
        update_display = TRUE;
        Display_Process();
    }
}

/*------------------------------------------------------------------------------
 * Switches_Process
 *------------------------------------------------------------------------------*/
void Switches_Process(void){
    Switch1_Process();
    Switch2_Process();
}

/*------------------------------------------------------------------------------
 * Enable / disable helpers
 *------------------------------------------------------------------------------*/
void enable_switch_SW1(void){
    P4IFG &= ~SW1;
    P4IE  |=  SW1;
}

void disable_switch_SW1(void){
    P4IE  &= ~SW1;
}

void enable_switch_SW2(void){
    P2IFG &= ~SW2;
    P2IE  |=  SW2;
}

void disable_switch_SW2(void){
    P2IE  &= ~SW2;
}
