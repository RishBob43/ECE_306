
#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

extern volatile unsigned int  update_display_count;
extern volatile unsigned char update_display;
extern volatile unsigned char display_changed;
extern volatile unsigned int  Time_Sequence;
extern volatile char          one_time;

volatile unsigned int msec_count = 0;

void Init_Timer_B0(void){
    TB0CTL   = TBSSEL__SMCLK;   // source = SMCLK (8MHz)
    TB0CTL  |= TBCLR;           // clear timer
    TB0CTL  |= ID__8;           // /8 -> 1,000,000 Hz
    TB0CCR0  = 1000;            // 1,000,000 / 1000 = 1ms tick
    TB0CCTL0 = CCIE;
    TB0CTL  |= MC__UP;
}

#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer0_B0_ISR(void){
    msec_count++;

    update_display_count++;
    if(update_display_count >= DISPLAY_UPDATE_MS){
        update_display       = TRUE;
        update_display_count = 0;
    }

    if((msec_count % TIME_SEQ_MS) == 0){
        Time_Sequence++;
        if(Time_Sequence >= TIME_SEQ_MAX) Time_Sequence = 0;
        one_time = 1;
    }
}

void Init_Timer_B3(void){
    TB3CTL   = TBSSEL__SMCLK;
    TB3CTL  |= TBCLR;
    TB3CTL  |= MC__UP;
    TB3CCR0  = WHEEL_PERIOD;
    TB3CCTL2 = OUTMOD_7;  TB3CCR2 = WHEEL_OFF;
    TB3CCTL3 = OUTMOD_7;  TB3CCR3 = WHEEL_OFF;
    TB3CCTL4 = OUTMOD_7;  TB3CCR4 = WHEEL_OFF;
    TB3CCTL5 = OUTMOD_7;  TB3CCR5 = WHEEL_OFF;
}

void Init_Timers(void){
    Init_Timer_B0();
    Init_Timer_B3();
}

void delay_ms(unsigned int ms){
    unsigned int start = msec_count;
    while((unsigned int)(msec_count - start) < ms);
}

void five_msec_sleep(unsigned int mult){ delay_ms(mult * 5); }

void usleep(unsigned int usec){
    volatile unsigned int i;
    for(i = 0; i < usec; i++) __delay_cycles(8);  // 8 cycles per us at 8MHz
}

void usleep10(unsigned int usec){ usleep(usec * TEN_USEC); }
