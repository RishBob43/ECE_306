/*------------------------------------------------------------------------------
 * File:        adc.c
 * Target:      MSP430FR2355
 *
 * Description: ADC initialization and interrupt service routine.
 *
 *              Timer B1 CCR0 triggers ADC conversions every 10 ms.
 *              The CCR0 ISR starts the ADC conversion sequence directly.
 *
 *              Single-channel single-conversion mode is used.  After each
 *              conversion completes the ISR stores the result, switches the
 *              mux to the next channel, and re-starts the converter.  The
 *              round-robin visits three channels every pass:
 *
 *                Pass 0  A2  P1.2  V_DETECT_L   (Left  IR detector)
 *                Pass 1  A3  P1.3  V_DETECT_R   (Right IR detector)
 *                Pass 2  A5  P1.5  V_THUMB       (Thumbwheel)
 *
 *              When pass 2 completes ADC_updated is set TRUE so the
 *              foreground display task knows fresh data is available.
 *
 *              ADC resolution: 10-bit (ADCRES_1).
 *              Results are right-shifted by 1 (÷2) to yield a 9-bit value
 *              stored in the detector globals.  ADC_Thumb is stored as-is
 *              (10-bit, 0-1023).
 *
 *              Port pin configuration for ADC inputs uses P1SELC (sets both
 *              SEL0 and SEL1 simultaneously) to disable the digital buffer
 *              and eliminate parasitic current on analog input pins.
 *
 * Globals defined here:
 *   ADC_Left_Detect   – scaled left  detector result  (10-bit >> 1 = 9-bit)
 *   ADC_Right_Detect  – scaled right detector result  (10-bit >> 1 = 9-bit)
 *   ADC_Thumb         – thumbwheel result             (10-bit, 0-1023)
 *   ADC_Channel       – round-robin index (0, 1, 2)
 *   ADC_updated       – flag: set when all three channels sampled
 *------------------------------------------------------------------------------*/

#include "ADC.h"
#include "msp430.h"
#include "functions.h"
#include "ports.h"
#include "macros.h"

/*------------------------------------------------------------------------------
 * Globals – owned by this translation unit, extern'd via ADC.h
 *------------------------------------------------------------------------------*/
volatile unsigned int  ADC_Left_Detect  = RESET_STATE;
volatile unsigned int  ADC_Right_Detect = RESET_STATE;
volatile unsigned int  ADC_Thumb        = RESET_STATE;
volatile unsigned char ADC_Channel      = RESET_STATE;
volatile unsigned char ADC_updated      = FALSE;

/*==============================================================================
 * Init_ADC
 *
 * Description: Configures the ADC module for single-channel single-conversion
 *              with a round-robin software channel switch in the ISR.
 *              10-bit resolution.  Conversions are triggered by Timer B1 CCR0
 *              (not started here – Init_Timer_B1() arms the first trigger).
 *
 * Register summary:
 *   ADCCTL0  – sample-hold time (16 clocks), MSC, ADC ON
 *   ADCCTL1  – ADCSC trigger, sampling timer, MODCLK, /1, single-ch
 *   ADCCTL2  – pre-divide /1, 10-bit resolution, unsigned binary, 200 ksps
 *   ADCMCTL0 – AVCC/AVSS reference, initial channel A2
 *   ADCIE    – conversion-complete interrupt enabled
 *
 * Globals changed:
 *   ADC_Channel  (reset to 0)
 *   ADC_updated  (reset to FALSE)
 *
 * Locals used: none
 *==============================================================================*/
void Init_ADC(void){

    /* Reset channel index and ready flag */
    ADC_Channel = RESET_STATE;
    ADC_updated = FALSE;

    /*--------------------------------------------------------------------------
     * ADCCTL0
     *------------------------------------------------------------------------*/
    ADCCTL0 = RESET_STATE;           /* Start from a known-zero state          */
    ADCCTL0 |= ADCSHT_2;             /* Sample-and-hold time = 16 ADC clocks   */
    ADCCTL0 |= ADCON;                /* ADC on                                  */

    /*--------------------------------------------------------------------------
     * ADCCTL1
     *------------------------------------------------------------------------*/
    ADCCTL1 = RESET_STATE;
    ADCCTL1 |= ADCSHS_0;             /* Trigger source: ADCSC bit               */
    ADCCTL1 |= ADCSHP;               /* SAMPCON from internal sampling timer    */
    ADCCTL1 &= ~ADCISSH;             /* Do NOT invert sample signal             */
    ADCCTL1 |= ADCDIV_0;             /* ADC clock divider /1                    */
    ADCCTL1 |= ADCSSEL_0;            /* ADC clock = MODCLK                      */
    ADCCTL1 |= ADCCONSEQ_0;          /* Single-channel single-conversion        */

    /*--------------------------------------------------------------------------
     * ADCCTL2  – 10-bit resolution
     *------------------------------------------------------------------------*/
    ADCCTL2 = RESET_STATE;
    ADCCTL2 |= ADCPDIV0;             /* Pre-divider /1                          */
    ADCCTL2 |= ADCRES_1;             /* 10-bit resolution (12-clock conversion) */
    ADCCTL2 &= ~ADCDF;               /* Unsigned binary format                  */
    ADCCTL2 &= ~ADCSR;               /* Up to 200 ksps sampling rate            */

    /*--------------------------------------------------------------------------
     * ADCMCTL0 – first channel = A2 (V_DETECT_L, P1.2)
     *------------------------------------------------------------------------*/
    ADCMCTL0  = RESET_STATE;
    ADCMCTL0 |= ADCSREF_0;           /* VR+ = AVCC,  VR– = AVSS                */
    ADCMCTL0 |= ADCINCH_2;           /* Initial channel A2 = V_DETECT_L        */

    /*--------------------------------------------------------------------------
     * Enable conversion-complete interrupt
     * Conversions are started by the TB1 CCR0 ISR every 10 ms.
     *------------------------------------------------------------------------*/
    ADCIE |= ADCIE0;                 /* Enable conversion-complete interrupt    */

    /*--------------------------------------------------------------------------
     * NOTE: ADCENC and ADCSC are NOT set here.
     *       The first conversion is started by the TB1 CCR0 ISR.
     *------------------------------------------------------------------------*/
}

/*==============================================================================
 * Init_Timer_B1
 *
 * Description: Configures Timer B1 in continuous mode with SMCLK/8/8 = 125 kHz
 *              (1 tick = 8 µs).
 *
 *              CCR0  fires every 10 ms to trigger one ADC conversion sequence.
 *                    10 ms = 125,000 * 0.010 = 1250 ticks
 *
 *              CCR1, CCR2, overflow – not used; left disabled.
 *
 *==============================================================================*/
void Init_Timer_B1(void){

    TB1CTL  = TBSSEL__SMCLK;         /* Clock source: SMCLK (8 MHz)            */
    TB1CTL |= MC__CONTINUOUS;        /* Continuous counting mode                */
    TB1CTL |= ID__8;                 /* Input divider /8                        */
    TB1EX0  = TBIDEX__8;            /* Expansion divider /8  -> 125 kHz        */
    TB1CTL |= TBCLR;                 /* Clear counter                           */

    /* CCR0 – fires every 10 ms to start ADC conversion */
    TB1CCR0  = TB1CCR0_INTERVAL;
    TB1CCTL0 &= ~CCIFG;             /* Clear any pending flag                  */
    TB1CCTL0 |=  CCIE;              /* Enable CCR0 interrupt                   */

    /* CCR1 – not used */
    TB1CCR1  = TB1CCR1_INTERVAL;
    TB1CCTL1 &= ~CCIFG;
    /* TB1CCTL1 |= CCIE; */         /* Disabled                                */

    /* CCR2 – not used */
    TB1CCR2  = TB1CCR2_INTERVAL;
    TB1CCTL2 &= ~CCIFG;
    /* TB1CCTL2 |= CCIE; */         /* Disabled                                */

    /* Overflow – not used */
    TB1CTL &= ~TBIE;
}

/*==============================================================================
 * TIMER1_B0_VECTOR  –  TB1 CCR0 ISR
 *
 * Fires every 10 ms.  Advances CCR0 by TB1CCR0_INTERVAL, enables the ADC,
 * and starts the first conversion in the round-robin sequence.
 * The ADC ISR handles the remaining channels automatically.
 *==============================================================================*/
#pragma vector = TIMER1_B0_VECTOR
__interrupt void TIMERB1_CCR0_ISR(void){

    TB1CCR0 += TB1CCR0_INTERVAL;    /* Schedule next 10 ms interrupt           */

    ADCCTL0 |= ADCENC;              /* Enable conversions                      */
    ADCCTL0 |= ADCSC;               /* Start first conversion (channel A2)     */
}

/*==============================================================================
 * ADC_VECTOR  –  ADC_ISR
 *
 * Interrupt source:  ADC conversion complete (ADCIFG in ADCIV)
 * Trigger:           ADCMEM0 loaded with new conversion result
 *
 * Round-robin channel sequence:
 *   ADC_Channel 0  ->  A2  V_DETECT_L   store -> ADC_Left_Detect,  next A3
 *   ADC_Channel 1  ->  A3  V_DETECT_R   store -> ADC_Right_Detect, next A5
 *   ADC_Channel 2  ->  A5  V_THUMB      store -> ADC_Thumb,
 *                                        set ADC_updated, reset to A2
 *
 * Results are right-shifted by 1 (÷2) for the detector channels to produce
 * a 9-bit value (0-511).  ADC_Thumb is stored as the full 10-bit result
 * (0-1023) for thumbwheel display.
 *
 * Globals changed:
 *   ADC_Left_Detect, ADC_Right_Detect, ADC_Thumb
 *   ADC_Channel
 *   ADC_updated
 *   ADCMCTL0  (channel mux changed each pass)
 *   ADCCTL0   (ENC toggled around mux change, SC re-armed)
 *   display_changed (set TRUE when full sequence completes)
 *
 * Locals used: none
 *==============================================================================*/

extern volatile unsigned char display_changed;

#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(void){

    switch(__even_in_range(ADCIV, ADCIV_ADCIFG)){

        case ADCIV_NONE:
            break;

        case ADCIV_ADCOVIFG:         /* Conversion overflow – result overwritten */
            break;

        case ADCIV_ADCTOVIFG:        /* Conversion time overflow                  */
            break;

        case ADCIV_ADCHIIFG:         /* Window comparator – high threshold        */
            break;

        case ADCIV_ADCLOIFG:         /* Window comparator – low threshold         */
            break;

        case ADCIV_ADCINIFG:         /* Window comparator – in-window             */
            break;

        case ADCIV_ADCIFG:           /* ADCMEM0 has a valid conversion result     */
            ADCCTL0 &= ~ADCENC;      /* Disable conversions before changing mux  */

            switch(ADC_Channel++){

                /*--------------------------------------------------------------
                 * Pass 0: result is from A2 (V_DETECT_L)
                 *         Shift right by 1 -> 9-bit result (0-511).
                 *         Switch mux to A3 (V_DETECT_R) for next pass.
                 *------------------------------------------------------------*/
                case ADC_CHANNEL_L_DETECT:
                    ADC_Left_Detect  = ADCMEM0 >> 1u; /* 10-bit >> 1 = 9-bit  */
                    ADCMCTL0 &= ~ADCINCH_2;   /* Clear A2 channel bits          */
                    ADCMCTL0 |=  ADCINCH_3;   /* Select A3 = V_DETECT_R         */
                    ADCCTL0  |=  ADCENC;       /* Re-enable                      */
                    ADCCTL0  |=  ADCSC;        /* Start next sample              */
                    break;

                /*--------------------------------------------------------------
                 * Pass 1: result is from A3 (V_DETECT_R)
                 *         Shift right by 1 -> 9-bit result.
                 *         Switch mux to A5 (V_THUMB) for next pass.
                 *------------------------------------------------------------*/
                case ADC_CHANNEL_R_DETECT:
                    ADC_Right_Detect = ADCMEM0 >> 1u; /* 10-bit >> 1 = 9-bit  */
                    ADCMCTL0 &= ~ADCINCH_3;   /* Clear A3 channel bits          */
                    ADCMCTL0 |=  ADCINCH_5;   /* Select A5 = V_THUMB            */
                    ADCCTL0  |=  ADCENC;       /* Re-enable                      */
                    ADCCTL0  |=  ADCSC;        /* Start next sample              */
                    break;

                /*--------------------------------------------------------------
                 * Pass 2: result is from A5 (V_THUMB)
                 *         Store full 10-bit thumbwheel value (no shift).
                 *         Reset mux to A2 for next round-robin cycle.
                 *         Signal foreground that a complete set is ready.
                 *         Set display_changed so the foreground refreshes LCD.
                 *------------------------------------------------------------*/
                case ADC_CHANNEL_THUMB:
                    ADC_Thumb        = ADCMEM0;        /* Full 10-bit, 0-1023   */
                    ADCMCTL0 &= ~ADCINCH_5;   /* Clear A5 channel bits          */
                    ADCMCTL0 |=  ADCINCH_2;   /* Return to A2 = V_DETECT_L      */
                    ADC_Channel      = RESET_STATE;   /* Reset round-robin index */
                    ADC_updated      = TRUE;           /* Notify foreground      */
                    display_changed  = TRUE;           /* Trigger LCD refresh    */
                    /* Leave ADCENC cleared; TB1 CCR0 ISR re-arms next cycle   */
                    break;

                default:
                    ADC_Channel = RESET_STATE;
                    break;
            }
            break;

        default:
            break;
    }
}
