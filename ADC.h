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
 *              Conversion is started in main / every 250 ms tick.
 *              Results are stored in the ADC ISR and displayed by
 *              Display_Process every quarter-second.
 *------------------------------------------------------------------------------*/

#ifndef ADC_H_
#define ADC_H_

#include <msp430.h>

/*------------------------------------------------------------------------------
 * ADC channel sequence indices
 *   The ISR uses ADC_Channel as an index into the round-robin:
 *     0 -> A2  (V_DETECT_L)
 *     1 -> A3  (V_DETECT_R)
 *     2 -> A5  (V_THUMB)
 *   After index 2 the counter resets to 0.
 *------------------------------------------------------------------------------*/
#define ADC_CHANNEL_L_DETECT    (0x00)   /* Index for left  detector channel   */
#define ADC_CHANNEL_R_DETECT    (0x01)   /* Index for right detector channel   */
#define ADC_CHANNEL_THUMB       (0x02)   /* Index for thumbwheel channel       */
#define ADC_CHANNEL_COUNT       (3u)     /* Total channels in round-robin      */

/*------------------------------------------------------------------------------
 * Raw 12-bit ADC result scaling
 *   The project divides raw 12-bit results by 4 to yield a 10-bit value
 *   (0-1023) that fits in 4 decimal digits on the LCD.
 *------------------------------------------------------------------------------*/
#define ADC_SCALE_SHIFT         (2u)     /* Right-shift by 2 = divide by 4     */

/*------------------------------------------------------------------------------
 * Line-detection threshold
 *   Empirically determined: when the emitter is ON and the detector is over
 *   a black surface the reflected IR is low, so ADCMEM0 reads HIGH (pull-up
 *   dominates).  Over white the reading is LOW (transistor conducts).
 *   Threshold is in scaled (÷4) ADC counts.
 *   Adjust during lab calibration.
 *------------------------------------------------------------------------------*/
#define BLACK_LINE_THRESHOLD    (700u)   /* Scaled ADC value -> black detected  */
#define WHITE_THRESHOLD         (400u)   /* Scaled ADC value -> white detected  */

/*------------------------------------------------------------------------------
 * Extern declarations for ADC result globals (defined in adc.c)
 *------------------------------------------------------------------------------*/
extern volatile unsigned int ADC_Left_Detect;
extern volatile unsigned int ADC_Right_Detect;
extern volatile unsigned int ADC_Thumb;
extern volatile unsigned char ADC_Channel;
extern volatile unsigned char ADC_updated;   /* Set TRUE when all 3 ch done   */

/*------------------------------------------------------------------------------
 * Function prototypes
 *------------------------------------------------------------------------------*/
void Init_ADC(void);

#endif /* ADC_H_ */
