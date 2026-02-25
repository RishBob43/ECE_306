//------------------------------------------------------------------------------
// timer.c
// Timer B0: 1ms tick  |  Timer B3: PWM motor control
//
// Clock chain for Timer B0:
//   SMCLK         = 500 kHz  (8MHz DCO / DIVS__16, set in clocks.c)
//   ID__4         = /4       -> timer clock = 125,000 Hz
//   TB0_1MS_COUNT = 125      -> 125,000 / 125 = 1,000 Hz (exactly 1ms per tick)
//------------------------------------------------------------------------------
#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

// Owned by LCD.obj - do NOT redefine, just reference
extern volatile unsigned int  update_display_count;
extern volatile unsigned char update_display;
extern volatile unsigned char display_changed;

// Owned by globals.c
extern volatile unsigned int  Time_Sequence;
extern volatile char          one_time;

// Owned by this file
volatile unsigned int msec_count = 0;

//------------------------------------------------------------------------------
// Init_Timer_B0 - exactly 1ms tick from 500kHz SMCLK
// SMCLK(500kHz) / ID__4 = 125kHz timer clock
// 125kHz / CCR0(125) = 1000 Hz = exactly 1ms tick
//------------------------------------------------------------------------------
void Init_Timer_B0(void) {
    TB0CTL   = TBSSEL__SMCLK;   // Source = SMCLK (500 kHz)
    TB0CTL  |= TBCLR;           // Clear timer and dividers
    TB0CTL  |= ID__4;           // Prescale /4 -> 125,000 Hz timer clock
    TB0CCR0  = TB0_1MS_COUNT;   // 125 counts = exactly 1ms
    TB0CCTL0 = CCIE;            // Enable CCR0 compare interrupt
    TB0CTL  |= MC__UP;          // Start timer in Up mode
}

//------------------------------------------------------------------------------
// Timer B0 ISR - fires every 1ms
//------------------------------------------------------------------------------
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer0_B0_ISR(void) {
    msec_count++;

    // Trigger display update every DISPLAY_UPDATE_MS ms (100ms)
    update_display_count++;
    if(update_display_count >= DISPLAY_UPDATE_MS) {
        update_display       = TRUE;
        update_display_count = 0;
    }

    // Increment Time_Sequence every TIME_SEQ_MS ms (50ms)
    if((msec_count % TIME_SEQ_MS) == 0) {
        Time_Sequence++;
        if(Time_Sequence >= TIME_SEQ_MAX) {
            Time_Sequence = 0;
        }
        one_time = 1;
    }
}

//------------------------------------------------------------------------------
// Init_Timer_B3 - PWM for motor control
// Uses SMCLK (500 kHz), period = WHEEL_PERIOD counts
//------------------------------------------------------------------------------
void Init_Timer_B3(void) {
    TB3CTL   = TBSSEL__SMCLK;           // Source = SMCLK (500 kHz)
    TB3CTL  |= TBCLR;                   // Clear timer
    TB3CTL  |= MC__UP;                  // Up mode
    TB3CCR0  = WHEEL_PERIOD;            // PWM period (defined in ports.h)
    TB3CCTL2 = OUTMOD_7;  TB3CCR2 = WHEEL_OFF;  // R_FORWARD  P6.1
    TB3CCTL3 = OUTMOD_7;  TB3CCR3 = WHEEL_OFF;  // L_FORWARD  P6.2
    TB3CCTL4 = OUTMOD_7;  TB3CCR4 = WHEEL_OFF;  // R_REVERSE  P6.3
    TB3CCTL5 = OUTMOD_7;  TB3CCR5 = WHEEL_OFF;  // L_REVERSE  P6.4
}

//------------------------------------------------------------------------------
// Init_Timers - called from main
//------------------------------------------------------------------------------
void Init_Timers(void) {
    Init_Timer_B0();
    Init_Timer_B3();
}

//------------------------------------------------------------------------------
// Blocking millisecond delay using msec_count tick
//------------------------------------------------------------------------------
void delay_ms(unsigned int ms) {
    unsigned int start = msec_count;
    while((unsigned int)(msec_count - start) < ms);
}

void five_msec_sleep(unsigned int mult) { delay_ms(mult * 5); }

//------------------------------------------------------------------------------
// usleep - busy-wait for 'usec' microseconds (MCLK = 8MHz)
//------------------------------------------------------------------------------
void usleep(unsigned int usec) {
    volatile unsigned int i;
    for(i = 0; i < usec; i++) __delay_cycles(CYCLES_PER_USEC);
}

//------------------------------------------------------------------------------
// usleep10 - busy-wait for usec * 10 microseconds
//------------------------------------------------------------------------------
void usleep10(unsigned int usec) { usleep(usec * TEN_USEC); }
