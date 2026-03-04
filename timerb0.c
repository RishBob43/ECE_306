/*------------------------------------------------------------------------------
 * File:        timerB0.c
 *
 * Description: Timer B0 initialization and interrupt service routines.
 *
 *              CCR0  - Fires every 200 ms
 *                      Toggles LCD_BACKLITE and sets the
 *                      update_display flag so Display_Process() may refresh
 *                      the LCD in the main loop foreground.
 *
 *              CCR1  - 100 ms debounce slice timer for SW1 (P4.1).
 *                      Enabled by the SW1 port interrupt; disabled after
 *                      DEBOUNCE_THRESHOLD slices (~1 second) have elapsed,
 *                      at which point SW1's port interrupt is re-enabled.
 *
 *              CCR2  - 100 ms debounce slice timer for SW2 (P2.3).
 *                      Same behavior as CCR1, applied to SW2.
 *
 *              Timer B3 – Motor PWM (unchanged from prior implementation).
 *
 * Timer Clock:  SMCLK (8 MHz) / ID__8 / TBIDEX__8 = 125,000 Hz
 *               1 tick = 8 µs
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "timerB0.h"


extern volatile unsigned char update_display;
extern volatile unsigned char display_changed;
extern volatile unsigned int  update_display_count;

/*------------------------------------------------------------------------------
 * Globals owned by this file (timerB0.c)
 *   debounce_flags      – bit 0 = SW1 debounce in progress
 *                          bit 1 = SW2 debounce in progress
 *   sw1_debounce_count  – counts 100 ms slices for SW1
 *   sw2_debounce_count  – counts 100 ms slices for SW2
 *------------------------------------------------------------------------------*/
volatile char         debounce_flags     = NO_DEBOUNCE_ACTIVE;
volatile unsigned int sw1_debounce_count = RESET_STATE;
volatile unsigned int sw2_debounce_count = RESET_STATE;

/*------------------------------------------------------------------------------
 * Init_Timer_B0
 *
 * Description: Configures Timer B0 in continuous mode with SMCLK / 8 / 8
 *              (125 kHz).  CCR0 is armed immediately for the 200 ms backlight
 *              blink.  CCR1 and CCR2 are loaded with their offset values but
 *              their interrupts are left DISABLED; the switch ISRs enable them
 *              on demand.
 *
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// Timer B0 initialization sets up both B0_0, B0_1-B0_2 and overflow
void Init_Timer_B0(void)
{
    TB0CTL = TBSSEL__SMCLK; // SMCLK source
    TB0CTL |= TBCLR; // Resets TB0R, clock divider, count direction
    TB0CTL |= MC__CONTINOUS;// Continuous up
    TB0CTL |= ID__8; // Divide clock by 8



    TB0EX0 = TBIDEX__8; // Divide clock by an additional 8
    TB0CCR0 = TB0CCR0_INTERVAL; // CCR0
    TB0CCTL0 |= CCIE; // CCR0 enable interrupt
    /* ---- CCR1: SW1 debounce slice (disabled until SW1 pressed) ---- */
    TB0CCR1  = TB0CCR1_DEBOUNCE_SLICE;            /* Preload offset             */
    TB0CCTL1 = RESET_STATE;                       /* Interrupt disabled         */

    /* ---- CCR2: SW2 debounce slice (disabled until SW2 pressed) ---- */
    TB0CCR2  = TB0CCR2_DEBOUNCE_SLICE;            /* Preload offset             */
    TB0CCTL2 = RESET_STATE;                       /* Interrupt disabled         */
    TB0CTL &= ~TBIE;
    TB0CTL &= ~TBIFG; // Disable Overflow Interrupt// Clear Overflow Interrupt flag
}
//------------------------------------------------------------------------------
/*------------------------------------------------------------------------------
 * Init_Timer_B3
 *
 * Description: Configures Timer B3 for motor PWM on P6.1-P6.4.
 *              Uses SMCLK, up mode, period = WHEEL_PERIOD.
 *              All four compare outputs use Reset/Set (OUTMOD_7).
 *
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void Init_Timer_B3(void){
    TB3CTL   = TBSSEL__SMCLK | TBCLR;
    TB3CTL  |= MC__UP;
    TB3CCR0  = WHEEL_PERIOD;
    TB3CCTL2 = OUTMOD_7;    TB3CCR2 = WHEEL_OFF;
    TB3CCTL3 = OUTMOD_7;    TB3CCR3 = WHEEL_OFF;
    TB3CCTL4 = OUTMOD_7;    TB3CCR4 = WHEEL_OFF;
    TB3CCTL5 = OUTMOD_7;    TB3CCR5 = WHEEL_OFF;
}

/*------------------------------------------------------------------------------
 * Init_Timers
 *
 * Description: Top-level timer initializer called from Init_Conditions.
 *
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void Init_Timers(void){
    Init_Timer_B0();
    Init_Timer_B3();
}

/*==============================================================================
 * TIMER0_B0_VECTOR  (CCR0 dedicated vector)
 *
 * Interrupt source:  Timer B0 CCR0 compare match
 * Trigger:           TB0R reaches TB0CCR0 value in continuous mode;
 *                    TB0CCR0 is advanced by TB0CCR0_INTERVAL each interrupt
 *                    to maintain a 200 ms period.
 *
 * Purpose:
 *   1. Advance TB0CCR0 for the next 200 ms period.
 *   2. Toggle LCD_BACKLITE output → backlight blinks at 2.5 Hz
 *      (on 200 ms, off 200 ms = one full cycle every 400 ms = 2.5 per second).
 *      Toggling is suppressed during an active debounce event; the backlight
 *      is forced OFF instead (per assignment requirement 12).
 *   3. Set update_display = TRUE so Display_Process() in main may call
 *      Display_Update() – enforcing the 200 ms minimum between LCD writes.
 *
 * Globals used:
 *   update_display    – set TRUE here; cleared by Display_Process()
 *   debounce_flags    – read to decide whether to suppress backlight
 *
 * Globals changed:
 *   update_display
 *   P6OUT (LCD_BACKLITE bit)
 *
 * Local variables:   none
 *============================================================================*/
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer0_B0_ISR(void){

    /* Advance CCR0 for the next 200 ms period (continuous mode offset) */
    TB0CCR0 += TB0CCR0_INTERVAL;

    /* Always signal that enough time has passed for a display refresh */
    update_display = TRUE;

    /* Toggle backlight only when no debounce is in progress. */
    if(debounce_flags == NO_DEBOUNCE_ACTIVE){
        P6OUT ^= LCD_BACKLITE;           /* Toggle -> 2.5 blinks per second    */
    } else {
        P6OUT &= ~LCD_BACKLITE;          /* Force backlight off during debounce */
    }
}

/*==============================================================================
 * TIMER0_B1_VECTOR  (CCR1, CCR2, and overflow shared vector)
 *
 * Interrupt source:  Timer B0 CCR1 or CCR2 compare match (TB0IV dispatch)
 * Trigger:           TB0R reaches TB0CCR1 (SW1 slice) or TB0CCR2 (SW2 slice)
 *                    in continuous mode.
 *
 * CCR1 – SW1 debounce slice handler
 *   - Advances TB0CCR1 by TB0CCR1_DEBOUNCE_SLICE for the next slice.
 *   - Increments sw1_debounce_count only while SW1 debounce is flagged.
 *   - When sw1_debounce_count reaches DEBOUNCE_THRESHOLD:
 *       * Clears the SW1 debounce-in-progress flag.
 *       * Resets sw1_debounce_count to zero.
 *       * Disables CCR1 interrupt
 *       * Re-enables SW1 port interrupt
 *       * Re-enables CCR0 backlight interrupt
 *
 * CCR2 – SW2 debounce slice handler (mirrors CCR1 logic for SW2)
 *
 * Globals used:
 *   debounce_flags        – read and written
 *   sw1_debounce_count    – incremented / cleared
 *   sw2_debounce_count    – incremented / cleared
 *
 * Globals changed:
 *   debounce_flags
 *   sw1_debounce_count
 *   sw2_debounce_count
 *   TB0CCTL0  (CCR0 re-enable)
 *   TB0CCTL1  (CCR1 disable)
 *   TB0CCTL2  (CCR2 disable)
 *   P4IE      (SW1 re-enable)
 *   P2IE      (SW2 re-enable)
 *
 * Local variables:   none
 *============================================================================*/
#pragma vector = TIMER0_B1_VECTOR
__interrupt void Timer0_B1_ISR(void){

    switch(TB0IV){

        /*----------------------------------------------------------------------
         * CCR1 compare match – SW1 debounce slice
         *--------------------------------------------------------------------*/
        case TB0IV_TB0CCR1:

            /* Advance CCR1 for next 100 ms slice */
            TB0CCR1 += TB0CCR1_DEBOUNCE_SLICE;

            /* Count only when SW1 debounce is flagged as active */
            if(debounce_flags & SW1_DEBOUNCE_ACTIVE_BIT){

                sw1_debounce_count++;

                if(sw1_debounce_count >= DEBOUNCE_THRESHOLD){

                    /* Debounce period expired --------------------------------*/

                    /* Clear SW1 debounce-in-progress flag */
                    debounce_flags &= ~SW1_DEBOUNCE_ACTIVE_BIT;

                    /* Reset count for next press */
                    sw1_debounce_count = RESET_STATE;

                    /* Disable CCR1 interrupt */
                    TB0CCTL1 &= CCR_INTERRUPT_DISABLE;

                    /* Re-enable SW1 port interrupt  */
                    P4IFG &= ~SW1;
                    P4IE  |=  SW1;

                    /* Re-enable backlight CCR0 only if SW2 debounce is
                     * also finished (requirement 13)                    */
                    if(!(debounce_flags & SW2_DEBOUNCE_ACTIVE_BIT)){
                        TB0CCTL0 |= CCR_INTERRUPT_ENABLE;
                    }
                }
            }
            break;

        /*----------------------------------------------------------------------
         * CCR2 compare match – SW2 debounce slice
         *--------------------------------------------------------------------*/
        case TB0IV_TB0CCR2:

            /* Advance CCR2 for next 100 ms slice */
            TB0CCR2 += TB0CCR2_DEBOUNCE_SLICE;

            /* Count only when SW2 debounce is flagged as active */
            if(debounce_flags & SW2_DEBOUNCE_ACTIVE_BIT){

                sw2_debounce_count++;

                if(sw2_debounce_count >= DEBOUNCE_THRESHOLD){

                    /* Debounce period expired --------------------------------*/

                    /* Clear SW2 debounce-in-progress flag */
                    debounce_flags &= ~SW2_DEBOUNCE_ACTIVE_BIT;

                    /* Reset count for next press */
                    sw2_debounce_count = RESET_STATE;

                    /* Disable CCR2 interrupt */
                    TB0CCTL2 &= CCR_INTERRUPT_DISABLE;

                    /* Re-enable SW2 port interrupt */
                    P2IFG &= ~SW2;
                    P2IE  |=  SW2;

                    /* Re-enable backlight CCR0 only if SW1 debounce is
                     * also finished                   */
                    if(!(debounce_flags & SW1_DEBOUNCE_ACTIVE_BIT)){
                        TB0CCTL0 |= CCR_INTERRUPT_ENABLE;
                    }
                }
            }
            break;

        /*----------------------------------------------------------------------
         * Timer overflow or unexpected source – no action
         *--------------------------------------------------------------------*/
        default:
            break;
    }
}


/*------------------------------------------------------------------------------
 * delay_ms
 * Description: Spin-wait for 'ms' milliseconds using __delay_cycles.
 * Globals used:  none
 * Locals used:   i (loop counter)
 *------------------------------------------------------------------------------*/
void delay_ms(unsigned int ms){
    volatile unsigned int i;
    for(i = RESET_STATE; i < ms; i++){
        __delay_cycles(CYCLES_PER_MS);
    }
}

/*------------------------------------------------------------------------------
 * five_msec_sleep
 * Description: Delay mult * 5 ms.
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void five_msec_sleep(unsigned int mult){
    delay_ms(mult * FIVE_MS_MULTIPLIER);
}

/*------------------------------------------------------------------------------
 * usleep
 * Description: Delay approximately 'usec' microseconds.
 *              Assumes MCLK = 8 MHz (8 cycles per microsecond).
 * Globals used:  none
 * Locals used:   i
 *------------------------------------------------------------------------------*/
void usleep(unsigned int usec){
    volatile unsigned int i;
    for(i = RESET_STATE; i < usec; i++){
        __delay_cycles(CYCLES_PER_USEC);
    }
}

/*------------------------------------------------------------------------------
 * usleep10
 * Description: Delay 'usec' * 10 microseconds.
 * Globals used:  none
 * Locals used:   none
 *------------------------------------------------------------------------------*/
void usleep10(unsigned int usec){
    usleep(usec * TEN_USEC);
}
