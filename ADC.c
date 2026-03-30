/*------------------------------------------------------------------------------
 * File:        adc.c
 * Target:      MSP430FR2355
 *
 * Description: ADC initialization and interrupt service routine.
 *
 *              Single-channel single-conversion mode is used.  After each
 *              conversion completes the ISR stores the result, switches the
 *              mux to the next channel, and re-starts the converter.  The
 *              round-robin visits three channels every pass:
 *
 *                Pass 0  A2  P1.2  V_DETECT_L
 *                Pass 1  A3  P1.3  V_DETECT_R
 *                Pass 2  A5  P1.5  V_THUMB
 *
 *              When pass 2 completes ADC_updated is set TRUE so the
 *              foreground display task knows fresh data is available.
 *
 *              Port pin configuration for ADC inputs uses P1SELC (sets both
 *              SEL0 and SEL1 simultaneously) to disable the digital buffer
 *              and eliminate parasitic current on analog input pins.
 *
 * Globals defined here:
 *   ADC_Left_Detect   – scaled left  detector result  (12-bit >> 2)
 *   ADC_Right_Detect  – scaled right detector result  (12-bit >> 2)
 *   ADC_Thumb         – scaled thumbwheel result      (12-bit >> 2)
 *   ADC_Channel       – round-robin index (0, 1, 2)
 *   ADC_updated       – flag: set when all three channels sampled
 *------------------------------------------------------------------------------*/

#include <ADC.h>
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
 *              Starts the first conversion on channel A2 (V_DETECT_L).
 *
 * Register summary:
 *   ADCCTL0  – sample-hold time (16 clocks), MSC, ADC ON
 *   ADCCTL1  – ADCSC trigger, sampling timer, MODCLK, /1, single-ch
 *   ADCCTL2  – pre-divide /1, 12-bit resolution, unsigned binary, 200 ksps
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
    ADCCTL0 |= ADCMSC;               /* Multiple sample and conversion          */
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
     * ADCCTL2
     *------------------------------------------------------------------------*/
    ADCCTL2 = RESET_STATE;
    ADCCTL2 |= ADCPDIV0;             /* Pre-divider /1                          */
    ADCCTL2 |= ADCRES_2;             /* 12-bit resolution                       */
    ADCCTL2 &= ~ADCDF;               /* Unsigned binary format                  */
    ADCCTL2 &= ~ADCSR;               /* Up to 200 ksps sampling rate            */

    /*--------------------------------------------------------------------------
     * ADCMCTL0 – first channel = A2 (V_DETECT_L, P1.2)
     *------------------------------------------------------------------------*/
    ADCMCTL0  = RESET_STATE;
    ADCMCTL0 |= ADCSREF_0;           /* VR+ = AVCC,  VR– = AVSS                */
    ADCMCTL0 |= ADCINCH_2;           /* Initial channel A2 = V_DETECT_L        */

    /*--------------------------------------------------------------------------
     * Enable interrupt and start first conversion
     *------------------------------------------------------------------------*/
    ADCIE  |= ADCIE0;                /* Enable conversion-complete interrupt    */
    ADCCTL0 |= ADCENC;               /* Enable conversions                      */
    ADCCTL0 |= ADCSC;                /* Start first conversion                  */
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
 * All results are right-shifted by ADC_SCALE_SHIFT (÷4) to map 12-bit
 * values into the 0-1023 range for 4-digit LCD display.
 *
 * Globals changed:
 *   ADC_Left_Detect, ADC_Right_Detect, ADC_Thumb
 *   ADC_Channel
 *   ADC_updated
 *   ADCMCTL0  (channel mux changed each pass)
 *   ADCCTL0   (ENC toggled around mux change, SC re-armed)
 *
 * Locals used: none
 *==============================================================================*/
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

//        case ADCIV_ADCIFG:           /* ADCMEM0 has a valid conversion result     */
//            ADCCTL0 &= ~ADCENC;      /* Disable conversions before changing mux  */
//
//            switch(ADC_Channel++){
//
//                /*--------------------------------------------------------------
//                 * Pass 0: result is from A2 (V_DETECT_L)
//                 *         Switch mux to A3 (V_DETECT_R) for next pass.
//                 *------------------------------------------------------------*/
//                case ADC_CHANNEL_L_DETECT:
//                    ADC_Left_Detect  = ADCMEM0 >> ADC_SCALE_SHIFT;
//                    ADCMCTL0 &= ~ADCINCH_2;   /* Clear A2 channel bits          */
//                    ADCMCTL0 |=  ADCINCH_3;   /* Select A3 = V_DETECT_R         */
//                    break;
//
//                /*--------------------------------------------------------------
//                 * Pass 1: result is from A3 (V_DETECT_R)
//                 *         Switch mux to A5 (V_THUMB) for next pass.
//                 *------------------------------------------------------------*/
//                case ADC_CHANNEL_R_DETECT:
//                    ADC_Right_Detect = ADCMEM0 >> ADC_SCALE_SHIFT;
//                    ADCMCTL0 &= ~ADCINCH_3;   /* Clear A3 channel bits          */
//                    ADCMCTL0 |=  ADCINCH_5;   /* Select A5 = V_THUMB            */
//                    break;
//
//                /*--------------------------------------------------------------
//                 * Pass 2: result is from A5 (V_THUMB)
//                 *         Reset mux to A2 for next round-robin cycle.
//                 *         Signal foreground that a complete set is ready.
//                 *------------------------------------------------------------*/
//                case ADC_CHANNEL_THUMB:
//                    ADC_Thumb        = ADCMEM0 >> ADC_SCALE_SHIFT;
//                    ADCMCTL0 &= ~ADCINCH_5;   /* Clear A5 channel bits          */
//                    ADCMCTL0 |=  ADCINCH_2;   /* Return to A2 = V_DETECT_L      */
//                    ADC_Channel      = RESET_STATE;   /* Reset round-robin index */
//                    ADC_updated      = TRUE;          /* Notify foreground       */
//                    break;
//
//                default:
//                    ADC_Channel = RESET_STATE;
//                    break;
//            }
//
//            ADCCTL0 |= ADCENC;       /* Re-enable conversions                   */
//            ADCCTL0 |= ADCSC;        /* Start next conversion                   */
//            break;

        default:
            break;
    }
}
