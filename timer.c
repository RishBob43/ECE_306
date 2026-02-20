//------------------------------------------------------------------------------
// Timer Configuration and Timing Functions
// Built with Code Composer Version: CCS12.4.0.00007_win64
//------------------------------------------------------------------------------

#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"

// External variables - defined in globals.c
extern volatile unsigned int Time_Sequence;
extern volatile char one_time;

// External variables - defined in LCD.c
extern volatile unsigned int update_display_count;
extern volatile unsigned char update_display;

// Local timing variables - defined here
volatile unsigned int msec_count = 0;
volatile unsigned int shape_timer = 0;

//------------------------------------------------------------------------------
// Timer B0 initialization for 1ms interrupts
//------------------------------------------------------------------------------
void Init_Timer_B0(void) {
    TB0CTL = TBSSEL__SMCLK;      // SMCLK source (8MHz)
    TB0CTL |= TBCLR;             // Resets TB0R, clock divider, count direction
    TB0CTL |= MC__UP;            // Up mode
    TB0CTL |= ID__8;             // Divide clock by 8
    TB0EX0 = TBIDEX__8;          // Divide clock by an additional 8
    // SMCLK/8/8 = 8MHz/64 = 125kHz
    // For 1ms interrupt: 125kHz / 1000 Hz = 125 counts
    TB0CCR0 = 125;               // CCR0 - 125 for true 1ms (1000 interrupts/sec)
    TB0CCTL0 |= CCIE;            // CCR0 enable interrupt
    TB0CCR1 = 62;                // CCR1 (half period)
    TB0CCTL1 &= ~CCIE;           // CCR1 disable interrupt
    TB0CCR2 = 62;                // CCR2 (half period)
    TB0CCTL2 &= ~CCIE;           // CCR2 disable interrupt
}

//------------------------------------------------------------------------------
// Timer B0 CCR0 Interrupt Service Routine - 1ms tick
//------------------------------------------------------------------------------
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer0_B0_ISR(void) {
    // Increment millisecond counter
    msec_count++;

    // Increment shape timer if active
    if(shape_timer > 0) {
        shape_timer++;
    }

    // Update display counter (every 100ms)
    update_display_count++;
    if(update_display_count >= 100) {
        update_display = TRUE;
        update_display_count = 0;
    }

    // Time sequence counter (every 50ms)
    if((msec_count % 50) == 0) {
        Time_Sequence++;
        if(Time_Sequence >= 250) {
            Time_Sequence = 0;
        }
        one_time = 1;
    }
}

//------------------------------------------------------------------------------
// Timer B3 initialization for PWM motor control
//------------------------------------------------------------------------------
void Init_Timer_B3(void) {
    TB3CTL = TBSSEL__SMCLK;      // SMCLK
    TB3CTL |= MC__UP;            // Up Mode
    TB3CTL |= TBCLR;             // Clear TAR

    TB3CCR0 = WHEEL_PERIOD;      // PWM Period

    // Right Forward PWM
    TB3CCTL2 = OUTMOD_7;         // CCR2 reset/set
    TB3CCR2 = WHEEL_OFF;         // CCR2 PWM duty cycle

    // Right Reverse PWM
    TB3CCTL3 = OUTMOD_7;         // CCR3 reset/set
    TB3CCR3 = WHEEL_OFF;         // CCR3 PWM duty cycle

    // Left Forward PWM
    TB3CCTL4 = OUTMOD_7;         // CCR4 reset/set
    TB3CCR4 = WHEEL_OFF;         // CCR4 PWM duty cycle

    // Left Reverse PWM
    TB3CCTL5 = OUTMOD_7;         // CCR5 reset/set
    TB3CCR5 = WHEEL_OFF;         // CCR5 PWM duty cycle
}

//------------------------------------------------------------------------------
// Initialize all timers
//------------------------------------------------------------------------------
void Init_Timers(void) {
    Init_Timer_B0();
    Init_Timer_B3();
}

//------------------------------------------------------------------------------
// Delay function in milliseconds
//------------------------------------------------------------------------------
void delay_ms(unsigned int ms) {
    unsigned int start_time = msec_count;
    while((msec_count - start_time) < ms) {
        // Wait
    }
}

//------------------------------------------------------------------------------
// Start shape timer
//------------------------------------------------------------------------------
void start_shape_timer(void) {
    shape_timer = 1;
}

//------------------------------------------------------------------------------
// Get elapsed time in milliseconds
//------------------------------------------------------------------------------
unsigned int get_shape_time(void) {
    return shape_timer;
}

//------------------------------------------------------------------------------
// Stop and reset shape timer
//------------------------------------------------------------------------------
void stop_shape_timer(void) {
    shape_timer = 0;
}

//------------------------------------------------------------------------------
// Five millisecond sleep function
//------------------------------------------------------------------------------
void five_msec_sleep(unsigned int msec) {
    delay_ms(msec * 5);
}

//------------------------------------------------------------------------------
// Microsecond sleep function (approximate)
//------------------------------------------------------------------------------
void usleep(unsigned int usec) {
    volatile unsigned int i;
    for(i = 0; i < usec; i++) {
        __delay_cycles(8);  // 8 cycles at 8MHz = 1us
    }
}

//------------------------------------------------------------------------------
// 10 microsecond sleep function
//------------------------------------------------------------------------------
void usleep10(unsigned int usec) {
    usleep(usec * 10);
}
