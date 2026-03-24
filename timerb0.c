/*------------------------------------------------------------------------------
 * File:        timerB0.c
 * Target:      MSP430FR2355
 *
 * Description: Timer B0 initialization and ISRs.
 *
 *              CCR0  - Fires every 200 ms, sets update_display.
 *              CCR1  - 100 ms debounce slice for SW1.
 *              CCR2  - 100 ms debounce slice for SW2.
 *              OVF   - TB0 overflow: drives DAC soft-start ramp via
 *                      DAC_Ramp_Step() until DAC_ready is set.
 *              Timer B3 - Motor PWM (CCR2-CCR5 for wheel speeds).
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "timerB0.h"
#include "dac.h"

extern volatile unsigned char update_display;
extern volatile unsigned char display_changed;
extern volatile unsigned int  update_display_count;

volatile char         debounce_flags     = NO_DEBOUNCE_ACTIVE;
volatile unsigned int sw1_debounce_count = RESET_STATE;
volatile unsigned int sw2_debounce_count = RESET_STATE;

/*------------------------------------------------------------------------------
 * Init_Timer_B0
 *------------------------------------------------------------------------------*/
void Init_Timer_B0(void){
    TB0CTL  = TBSSEL__SMCLK;
    TB0CTL |= TBCLR;
    TB0CTL |= MC__CONTINOUS;
    TB0CTL |= ID__8;

    TB0EX0  = TBIDEX__8;

    TB0CCR0  = TB0CCR0_INTERVAL;
    TB0CCTL0 |= CCIE;

    TB0CCR1  = TB0CCR1_DEBOUNCE_SLICE;
    TB0CCTL1 = RESET_STATE;

    TB0CCR2  = TB0CCR2_DEBOUNCE_SLICE;
    TB0CCTL2 = RESET_STATE;

    /* Overflow interrupt is enabled by Init_DAC() when the DAC ramp starts.
     * It is disabled automatically by DAC_Ramp_Step() when ramp completes.
     * Leave TBIE cleared here so no spurious overflow fires before Init_DAC. */
    TB0CTL &= ~TBIE;
    TB0CTL &= ~TBIFG;
}

/*------------------------------------------------------------------------------
 * Init_Timer_B3
 *   Motor PWM: UP mode, WHEEL_PERIOD, OUTMOD_7 (reset/set) on CCR2-CCR5.
 *   All duty cycles start at WHEEL_OFF (motors stopped).
 *------------------------------------------------------------------------------*/
void Init_Timer_B3(void){
    TB3CTL   = TBSSEL__SMCLK;
    TB3CTL  |= MC__UP;
    TB3CTL |= TBCLR; // Clear TAR
    TB3CCR0  = WHEEL_PERIOD;
    TB3CCTL2 = OUTMOD_7;    TB3CCR2 = WHEEL_OFF;
    TB3CCTL3 = OUTMOD_7;    TB3CCR3 = WHEEL_OFF;
    TB3CCTL4 = OUTMOD_7;    TB3CCR4 = WHEEL_OFF;
    TB3CCTL5 = OUTMOD_7;    TB3CCR5 = WHEEL_OFF;
}

/*------------------------------------------------------------------------------
 * Init_Timers
 *------------------------------------------------------------------------------*/
void Init_Timers(void){
    Init_Timer_B0();
    Init_Timer_B3();
}

/*==============================================================================
 * TIMER0_B0_VECTOR  –  CCR0
 * Fires every 200 ms. Sets update_display for the foreground.
 *==============================================================================*/
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer0_B0_ISR(void){
    TB0CCR0 += TB0CCR0_INTERVAL;
    update_display = TRUE;
}

/*==============================================================================
 * TIMER0_B1_VECTOR  –  CCR1, CCR2, overflow (IV=14)
 *
 * CCR1 / CCR2 – switch debounce slices (re-enable switch interrupt after 1 s)
 * Overflow    – DAC soft-start ramp step; auto-disables when ramp is complete
 *==============================================================================*/
#pragma vector = TIMER0_B1_VECTOR
__interrupt void Timer0_B1_ISR(void){

    switch(TB0IV){

        /*----------------------------------------------------------------------
         * CCR1 – SW1 debounce
         *--------------------------------------------------------------------*/
        case TB0IV_TB0CCR1:
            TB0CCR1 += TB0CCR1_DEBOUNCE_SLICE;

            if(debounce_flags & SW1_DEBOUNCE_ACTIVE_BIT){
                sw1_debounce_count++;
                if(sw1_debounce_count >= DEBOUNCE_THRESHOLD){
                    debounce_flags     &= ~SW1_DEBOUNCE_ACTIVE_BIT;
                    sw1_debounce_count  = RESET_STATE;
                    TB0CCTL1           &= CCR_INTERRUPT_DISABLE;
                    P4IFG &= ~SW1;
                    P4IE  |=  SW1;
                }
            }
            break;

        /*----------------------------------------------------------------------
         * CCR2 – SW2 debounce
         *--------------------------------------------------------------------*/
        case TB0IV_TB0CCR2:
            TB0CCR2 += TB0CCR2_DEBOUNCE_SLICE;

            if(debounce_flags & SW2_DEBOUNCE_ACTIVE_BIT){
                sw2_debounce_count++;
                if(sw2_debounce_count >= DEBOUNCE_THRESHOLD){
                    debounce_flags     &= ~SW2_DEBOUNCE_ACTIVE_BIT;
                    sw2_debounce_count  = RESET_STATE;
                    TB0CCTL2           &= CCR_INTERRUPT_DISABLE;
                    P2IFG &= ~SW2;
                    P2IE  |=  SW2;
                }
            }
            break;

        /*----------------------------------------------------------------------
         * Overflow (TB0IV = 14) – DAC soft-start ramp
         *   DAC_Ramp_Step() decrements SAC3DAT each overflow until the
         *   operating voltage is reached, then disables TBIE itself.
         *--------------------------------------------------------------------*/
        case TB0IV_TB0IFG:
            DAC_Ramp_Step();
            break;

        default:
            break;
    }
}

/*------------------------------------------------------------------------------
 * Delay utilities
 *------------------------------------------------------------------------------*/
void delay_ms(unsigned int ms){
    volatile unsigned int i;
    for(i = RESET_STATE; i < ms; i++){
        __delay_cycles(CYCLES_PER_MS);
    }
}

void five_msec_sleep(unsigned int mult){
    delay_ms(mult * FIVE_MS_MULTIPLIER);
}

void usleep(unsigned int usec){
    volatile unsigned int i;
    for(i = RESET_STATE; i < usec; i++){
        __delay_cycles(CYCLES_PER_USEC);
    }
}

void usleep10(unsigned int usec){
    usleep(usec * TEN_USEC);
}
