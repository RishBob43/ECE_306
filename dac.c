/*------------------------------------------------------------------------------
 * File:        dac.c
 * Target:      MSP430FR2355
 *
 * Description: SAC3 / LT1935 DAC motor voltage controller.
 *
 *              Init_DAC() configures SAC3 in buffer mode, sets the starting
 *              DAC value to DAC_BEGIN (~2 V), enables the LT1935 via P2.5
 *              (DAC_ENB), and enables the TB0 overflow interrupt to drive the
 *              soft-start voltage ramp.
 *
 *              DAC_Ramp_Step() is called from the TB0 overflow ISR each time
 *              the timer overflows.  It decrements DAC_data by DAC_RAMP_STEP
 *              until DAC_data <= DAC_LIMIT, then sets DAC_ADJUST, disables the
 *              overflow interrupt, and asserts DAC_ready.
 *
 *              DAC_Set() writes a clamped value directly to SAC3DAT for
 *              runtime speed control.
 *
 * Hardware connections:
 *   P3.5  DAC_CNTL – configured via P3SELC for DAC operation
 *   P2.5  DAC_ENB  – HIGH enables the LT1935 buck-boost converter
 *   SAC3  OA0O pin – DAC output drives LT1935 FB_DAC input
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include "functions.h"
#include "ports.h"
#include "macros.h"
#include "dac.h"

/*------------------------------------------------------------------------------
 * Globals – defined here, extern'd via dac.h
 *------------------------------------------------------------------------------*/
volatile unsigned int  DAC_data  = DAC_BEGIN;
volatile unsigned char DAC_ready = FALSE;

/*==============================================================================
 * Init_DAC
 *
 * Description: Configures SAC3 for DAC buffer mode.  Begins the soft-start
 *              ramp by enabling the TB0 overflow interrupt.  The ramp itself
 *              runs in DAC_Ramp_Step() called from the ISR.
 *
 *              Call after Init_Timers() so TB0 is already running.
 *
 * Globals changed:
 *   DAC_data  (set to DAC_BEGIN)
 *   DAC_ready (set to FALSE – cleared to TRUE by DAC_Ramp_Step when done)
 *   SAC3DAT, SAC3DAC, SAC3OA, SAC3PGA (hardware registers)
 *   P3SELC, P2OUT (port registers)
 *   TB0CTL (overflow interrupt enabled)
 *==============================================================================*/
void Init_DAC(void){

    DAC_data  = DAC_BEGIN;
    DAC_ready = FALSE;

    /*--------------------------------------------------------------------------
     * Configure SAC3 in DAC buffer mode
     * SAC3DAC: reference = AVCC, latch on write to DACDAT
     * SAC3OA:  negative/positive mux enabled, DAC as positive source,
     *          negative pin input, low-power mode
     * SAC3PGA: buffer mode (MSEL_1)
     *------------------------------------------------------------------------*/
    SAC3DAC  = DACSREF_0;            /* VCC as DAC reference                   */
    SAC3DAC |= DACLSEL_0;            /* Latch DAC when DACDAT written          */

    SAC3OA   = NMUXEN;               /* Enable SAC negative input MUX          */
    SAC3OA  |= PMUXEN;               /* Enable SAC positive input MUX          */
    SAC3OA  |= PSEL_1;               /* 12-bit DAC as positive source          */
    SAC3OA  |= NSEL_1;               /* Negative pin input selected            */
    SAC3OA  |= OAPM;                 /* Low speed / low power OA mode          */
    SAC3PGA  = MSEL_1;               /* OA configured as unity-gain buffer     */
    SAC3OA  |= SACEN;                /* Enable SAC                             */
    SAC3OA  |= OAEN;                 /* Enable OA                              */

    /*--------------------------------------------------------------------------
     * Configure P3.5 (DAC_CNTL) for DAC analog function
     * P3SELC sets both SEL0 and SEL1 simultaneously, enabling the DAC pin.
     *------------------------------------------------------------------------*/
    P3OUT  &= ~DAC_CNTL;             /* Drive low before switching function    */
    P3DIR  &= ~DAC_CNTL;             /* Set as input before SELC               */
    P3SELC |=  DAC_CNTL;             /* Enable DAC analog function on P3.5     */

    /*--------------------------------------------------------------------------
     * Write initial DAC value and enable DAC output
     *------------------------------------------------------------------------*/
    SAC3DAT  = DAC_data;             /* Load initial value (~2 V)              */
    SAC3DAC |= DACEN;                /* Enable DAC output                      */

    /*--------------------------------------------------------------------------
     * Enable LT1935 buck-boost converter via P2.5 (DAC_ENB = HIGH)
     *------------------------------------------------------------------------*/
    P2OUT |= DAC_ENB;                /* Assert DAC_ENB -> converter ON         */

    /*--------------------------------------------------------------------------
     * Enable TB0 overflow interrupt to run the soft-start ramp.
     * The RED_LED lights to indicate the ramp is in progress.
     *------------------------------------------------------------------------*/
    P1OUT |=  RED_LED;               /* RED_LED ON: ramp in progress           */
    TB0CTL |= TBIE;                  /* Enable TB0 overflow interrupt          */
}

/*==============================================================================
 * DAC_Ramp_Step
 *
 * Description: Called from the TB0 overflow ISR on each timer overflow.
 *              Decrements DAC_data by DAC_RAMP_STEP (raising motor voltage).
 *              When DAC_data reaches or falls below DAC_LIMIT:
 *                - Sets DAC_data to DAC_ADJUST (settles at operating voltage)
 *                - Disables the TB0 overflow interrupt
 *                - Sets DAC_ready = TRUE to notify the foreground
 *                - Turns off RED_LED
 *
 * Globals changed: DAC_data, DAC_ready, SAC3DAT, TB0CTL, P1OUT
 *==============================================================================*/
void DAC_Ramp_Step(void){

    if(DAC_data > DAC_RAMP_STEP){
        DAC_data -= DAC_RAMP_STEP;
    } else {
        DAC_data = DAC_MIN_VALUE;
    }

    SAC3DAT = DAC_data;              /* Update converter voltage               */

    if(DAC_data <= DAC_LIMIT){
        DAC_data  = DAC_ADJUST;      /* Settle at operating point              */
        SAC3DAT   = DAC_data;
        TB0CTL   &= ~TBIE;           /* Disable overflow interrupt             */
        DAC_ready  = TRUE;           /* Signal foreground: voltage is ready    */
        P1OUT     &= ~RED_LED;       /* RED_LED OFF: ramp complete             */
    }
}

/*==============================================================================
 * DAC_Set
 *
 * Description: Writes a clamped 12-bit value to SAC3DAT for runtime speed
 *              control.  Values outside [DAC_MIN_VALUE, DAC_MAX_VALUE] are
 *              clamped silently.
 *
 * Parameters:
 *   value  desired DAC register value (0 = max voltage, 4095 = min voltage)
 *
 * Globals changed: DAC_data, SAC3DAT
 *==============================================================================*/
void DAC_Set(unsigned int value){

    if(value > DAC_MAX_VALUE){ value = DAC_MAX_VALUE; }

    DAC_data = value;
    SAC3DAT  = DAC_data;
}
