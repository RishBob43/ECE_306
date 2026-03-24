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
 *
 *   These are also defined in macros.h – the guards below prevent a
 *   redefinition error if both headers are included in the same translation
 *   unit.
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
 * Raw 12-bit ADC result scaling
 *   The project divides raw 12-bit results by 4 to yield a 10-bit value
 *   (0-1023) that fits in 4 decimal digits on the LCD.
 *------------------------------------------------------------------------------*/
#ifndef ADC_SCALE_SHIFT
#define ADC_SCALE_SHIFT         (2u)     /* Right-shift by 2 = divide by 4     */
#endif

/*------------------------------------------------------------------------------
 * Line-detection thresholds
 *   Defined in macros.h.  The #ifndef guards here are a safety net only –
 *   macros.h should always be included before ADC.h so these never trigger.
 *------------------------------------------------------------------------------*/
#ifndef BLACK_LINE_THRESHOLD
#define BLACK_LINE_THRESHOLD    (650u)   /* Scaled ADC value -> black detected  */
#endif

#ifndef WHITE_THRESHOLD
#define WHITE_THRESHOLD         (125u)   /* Scaled ADC value -> white detected  */
#endif

/*------------------------------------------------------------------------------
 * Extern declarations for ADC result globals (defined in adc.c)
 *------------------------------------------------------------------------------*/
extern volatile unsigned int  ADC_Left_Detect;
extern volatile unsigned int  ADC_Right_Detect;
extern volatile unsigned int  ADC_Thumb;
extern volatile unsigned char ADC_Channel;
extern volatile unsigned char ADC_updated;   /* Set TRUE when all 3 ch done   */

/*------------------------------------------------------------------------------
 * Function prototypes
 *------------------------------------------------------------------------------*/
void Init_ADC(void);

#endif /* ADC_H_ */
