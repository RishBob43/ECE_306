/*------------------------------------------------------------------------------
 * File:        ADC.h
 * Target:      MSP430FR2355
 *
 * Description: Header for ADC initialization and interrupt service routine.
 *              Declares the three ADC result globals and the channel counter
 *              used by the round-robin single-channel conversion sequence:
 *                Channel A2  -> V_DETECT_L  (P1.2)
 *                Channel A3  -> V_DETECT_R  (P1.3)
 *                Channel A5  -> V_THUMB     (P1.5)
 *
 *              Timer B1 CCR0 triggers conversions every 10 ms:
 *                Timer clock: SMCLK / 8 / 8 = 125,000 Hz (1 tick = 8 µs)
 *                10 ms period: 0.010 s × 125,000 = 1250 ticks
 *
 *              CCR0 ISR enables ADCENC and sets ADCSC to start the first
 *              conversion.  The ADC ISR round-robins through all three
 *              channels and leaves ADCENC cleared after the last channel so
 *              no conversion fires until the next CCR0 interrupt.
 *
 *              ADC resolution: 10-bit (ADCRES_1).
 *              Detector results are shifted right by 1 (÷2) -> 9-bit (0-511).
 *              Thumbwheel result is kept at full 10-bit (0-1023).
 *
 *------------------------------------------------------------------------------*/

#ifndef ADC_H_
#define ADC_H_

#include "msp430.h"
#include "macros.h"

/*------------------------------------------------------------------------------
 * ADC channel sequence indices
 *   The ISR uses ADC_Channel as an index into the round-robin:
 *     0 -> A2  (V_DETECT_L)
 *     1 -> A3  (V_DETECT_R)
 *     2 -> A5  (V_THUMB)
 *   After index 2 the counter resets to 0.
 *------------------------------------------------------------------------------*/
#ifndef ADC_CHANNEL_L_DETECT
#define ADC_CHANNEL_L_DETECT    (0x00)   /* Index for left  detector channel   */
#endif

#ifndef ADC_CHANNEL_R_DETECT
#define ADC_CHANNEL_R_DETECT    (0x01)   /* Index for right detector channel   */
#endif

#ifndef ADC_CHANNEL_THUMB
#define ADC_CHANNEL_THUMB       (0x02)   /* Index for thumbwheel channel       */
#endif

#ifndef ADC_CHANNEL_COUNT
#define ADC_CHANNEL_COUNT       (3u)     /* Total channels in round-robin      */
#endif

/*------------------------------------------------------------------------------
 * Raw ADC result scaling
 *   10-bit ADC (0-1023).  Detector results are shifted right 1 -> 9-bit (0-511).
 *------------------------------------------------------------------------------*/
#ifndef ADC_SCALE_SHIFT
#define ADC_SCALE_SHIFT         (1u)     /* Right-shift by 1 = divide by 2     */
#endif

/*------------------------------------------------------------------------------
 * Timer B1 CCR0 interval constant
 *   Timer clock: SMCLK / 8 / 8 = 125,000 Hz  (1 tick = 8 µs)
 *
 *   10 ms ADC trigger period:  0.010 s × 125,000  =  1250 ticks
 *
 *   CCR1 / CCR2 placeholders (unused) default to 1250 ticks.
 *------------------------------------------------------------------------------*/
#define TB1CCR0_INTERVAL        (1250u)  /* 10 ms – ADC conversion trigger     */
#define TB1CCR1_INTERVAL        (1250u)  /* CCR1 placeholder (not used)        */
#define TB1CCR2_INTERVAL        (1250u)  /* CCR2 placeholder (not used)        */

/*------------------------------------------------------------------------------
 * Line-detection thresholds
 *   Scaled for 9-bit detector results (0-511 after >>1).
 *   BLACK_LINE_THRESHOLD is the static fallback; runtime uses g_threshold.
 *   WHITE_THRESHOLD is the lower bound for "definitely white".
 *
 *   Adjust on bench to match actual sensor readings.
 *------------------------------------------------------------------------------*/
#ifndef BLACK_LINE_THRESHOLD
#define BLACK_LINE_THRESHOLD    (325u)   /* 9-bit: black detected              */
#endif

#ifndef WHITE_THRESHOLD
#define WHITE_THRESHOLD         (62u)    /* 9-bit: white detected              */
#endif

/*------------------------------------------------------------------------------
 * Extern declarations for ADC result globals (defined in adc.c)
 *------------------------------------------------------------------------------*/
extern volatile unsigned int  ADC_Left_Detect;
extern volatile unsigned int  ADC_Right_Detect;
extern volatile unsigned int  ADC_Thumb;
extern volatile unsigned char ADC_Channel;
extern volatile unsigned char ADC_updated;    /* Set TRUE when all 3 ch done   */

/*------------------------------------------------------------------------------
 * Function prototypes
 *------------------------------------------------------------------------------*/
void Init_ADC(void);
void Init_Timer_B1(void);

#endif /* ADC_H_ */
