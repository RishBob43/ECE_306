/*------------------------------------------------------------------------------
 * File:        dac.h
 * Target:      MSP430FR2355
 *
 * Description: Header for DAC / LT1935 buck-boost motor voltage controller.
 *
 *              The SAC3 / DAC drives the FB_DAC pin of the LT1935 converter.
 *              A higher DAC value  -> lower FB voltage -> lower output voltage
 *              A lower  DAC value  -> higher FB voltage -> higher output voltage
 *
 *              DAC ramp on startup:
 *                Init_DAC() sets DAC_data = DAC_BEGIN (2 V proxy), enables
 *                the overflow interrupt on TB0, and turns on the DAC.
 *                The TB0 overflow ISR decrements DAC_data by DAC_RAMP_STEP
 *                each overflow until DAC_data <= DAC_LIMIT, at which point it
 *                is set to DAC_ADJUST and the overflow interrupt is disabled.
 *
 *              Runtime control:
 *                DAC_Set(value)  – write a new value directly (0-4095).
 *                DAC_Ramp_Up()   – call from TB0 overflow; handles the soft-
 *                                  start ramp and auto-disables when done.
 *
 *              Motor voltage operating points (measured, AVCC reference):
 *                DAC value 2725 ≈ 2.0 V  (start / soft-ramp begin)
 *                DAC value 1400 ≈ 4.32 V  (DAC_CRUISE - line follow speed)
 *                DAC value 1000 ≈ 5.61 V
 *                DAC value  850 ≈ 6.08 V  (full speed)
 *                DAC value  715 ≈ 6.5  V  (max, use with caution)
 *------------------------------------------------------------------------------*/

#ifndef DAC_H_
#define DAC_H_

#include "msp430.h"
#include "macros.h"

/*------------------------------------------------------------------------------
 * DAC operating points
 *   Higher value  = lower output voltage (inverse relationship through LT1935)
 *   Lower  value  = higher output voltage
 *------------------------------------------------------------------------------*/
#define DAC_BEGIN               (2725u)  /* ~2.0 V – soft-start initial value  */
#define DAC_CRUISE              (1200u)  /* ~4.3 V – controlled line-follow speed */
#define DAC_LIMIT               (850u)   /* ~6.08 V – full speed ceiling        */
#define DAC_ADJUST              (DAC_LIMIT) /* Settle point after ramp completes */
#define DAC_RAMP_STEP           (100u)   /* Decrement per TB0 overflow tick     */
#define DAC_MAX_VALUE           (4095u)  /* 12-bit ceiling                      */
#define DAC_MIN_VALUE           (0u)     /* 12-bit floor                        */

/*------------------------------------------------------------------------------
 * Motor PWM base speed and PID limits (used by line_follow_pid in main.c)
 *
 *   PID_BASE_SPEED  – CCR value for both wheels when error = 0
 *   PID_MIN_SPEED   – floor: prevents a wheel from stopping or reversing
 *   PID_MAX_SPEED   – ceiling: prevents CCR from exceeding WHEEL_PERIOD
 *
 * These are kept here (not macros.h) because they are tightly coupled to the
 * DAC voltage level.  If DAC_CRUISE changes, re-evaluate PID_BASE_SPEED.
 *------------------------------------------------------------------------------*/
#define PID_BASE_SPEED          (6000u)  /* TB3CCR duty at straight-ahead       */
#define PID_MIN_SPEED           (1500u)  /* minimum CCR – keep wheels turning   */
#define PID_MAX_SPEED           (12000u) /* maximum CCR – below WHEEL_PERIOD    */

/*------------------------------------------------------------------------------
 * Extern: DAC_data is modified by both Init_DAC() and the TB0 overflow ISR.
 * Declared volatile because the ISR writes it asynchronously.
 *------------------------------------------------------------------------------*/
extern volatile unsigned int DAC_data;

/*------------------------------------------------------------------------------
 * Flag: set TRUE by the ramp ISR when DAC has settled at operating voltage.
 * Foreground code polls this before enabling motors.
 *------------------------------------------------------------------------------*/
extern volatile unsigned char DAC_ready;

/*------------------------------------------------------------------------------
 * Function prototypes
 *------------------------------------------------------------------------------*/
void Init_DAC(void);
void DAC_Set(unsigned int value);
void DAC_Ramp_Step(void);    /* called from TB0 overflow ISR in timerB0.c     */

#endif /* DAC_H_ */
