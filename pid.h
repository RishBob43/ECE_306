/*------------------------------------------------------------------------------
 * File:        pid.h
 * Target:      MSP430FR2355
 *
 * Description: Lightweight signed-integer PID controller for line following.
 *
 *              The PID error signal is the difference between the right and
 *              left detector ADC readings (both scaled 0-1023 after >>2):
 *
 *                error = (int)ADC_Right_Detect - (int)ADC_Left_Detect
 *
 *              When the car drifts RIGHT off the line the right detector sees
 *              more black (higher value) so error > 0  -> steer left.
 *              When it drifts LEFT  error < 0           -> steer right.
 *
 *              Output: signed correction applied symmetrically to the two
 *              TB3CCR wheel duty-cycle registers:
 *
 *                TB3CCR_RIGHT -= correction   (right wheel slows on right drift)
 *                TB3CCR_LEFT  += correction   (left  wheel speeds on right drift)
 *
 *              All three gains are stored as scaled integers to avoid floating
 *              point on the MSP430.  Divide by PID_GAIN_SCALE after each
 *              multiplication.
 *
 * Usage:
 *   PID_State pid;
 *   PID_Init(&pid);
 *   int correction = PID_Update(&pid, error);
 *------------------------------------------------------------------------------*/

#ifndef PID_H_
#define PID_H_

#include "macros.h"

/*------------------------------------------------------------------------------
 * Gain scaling factor
 *   All gains are stored multiplied by PID_GAIN_SCALE so that fractional
 *   gains can be represented with integer arithmetic.
 *   e.g. Kp = 0.5  ->  PID_KP = 5,  PID_GAIN_SCALE = 10
 *------------------------------------------------------------------------------*/
#define PID_GAIN_SCALE          (10)     /* divide by 10 after multiply         */

/*------------------------------------------------------------------------------
 * PID gains (multiplied by PID_GAIN_SCALE)
 *
 *   PID_KP  – proportional: main steering response
 *   PID_KI  – integral:     corrects steady-state drift over many ticks
 *   PID_KD  – derivative:   damps oscillation / overshoot
 *
 * Starting values for a 36-inch circle at ~4-5 V motor supply:
 *   Kp = 1.5 (PID_KP = 15)
 *   Ki = 0.0 (PID_KI =  0)  <- add slowly; integral windup is a real risk
 *   Kd = 0.5 (PID_KD =  5)
 *
 * Tune on the bench:
 *   1. Set Ki=0, Kd=0. Increase Kp until car oscillates, then back off ~30%.
 *   2. Add Kd in small steps until oscillation damps cleanly.
 *   3. Add Ki last, in very small steps, only if steady offset persists.
 *------------------------------------------------------------------------------*/
#define PID_KP                  (8000)     /* Proportional gain * GAIN_SCALE      */
#define PID_KI                  (0)      /* Integral     gain * GAIN_SCALE      */
#define PID_KD                  (0)      /* Derivative   gain * GAIN_SCALE      */

/*------------------------------------------------------------------------------
 * Anti-windup: clamp the integral accumulator to ±PID_INTEGRAL_LIMIT.
 * Prevents the integral term from growing unbounded during long off-line
 * excursions (e.g. while aligned or intercepting).
 *------------------------------------------------------------------------------*/
#define PID_INTEGRAL_LIMIT      (5000)

/*------------------------------------------------------------------------------
 * Output clamp: correction is clamped to ±PID_OUTPUT_LIMIT before being
 * applied to the CCR registers.  Set to half of PID_BASE_SPEED so a full
 * correction can swing one wheel to zero without going negative.
 *------------------------------------------------------------------------------*/
#define PID_OUTPUT_LIMIT        (6000)

/*------------------------------------------------------------------------------
 * PID_State – all persistent state for one controller instance
 *------------------------------------------------------------------------------*/
typedef struct {
    int integral;       /* Running sum of error * PID_GAIN_SCALE               */
    int prev_error;     /* Error from the previous PID_Update() call           */
} PID_State;

/*------------------------------------------------------------------------------
 * PID_Init – reset controller state (call before starting a new run)
 *------------------------------------------------------------------------------*/
static inline void PID_Init(PID_State *pid){
    pid->integral   = RESET_STATE;
    pid->prev_error = RESET_STATE;
}

/*------------------------------------------------------------------------------
 * PID_Update – compute one control step
 *
 * Parameters:
 *   pid    pointer to persistent state structure
 *   error  signed error this iteration (right_adc - left_adc)
 *
 * Returns:
 *   signed correction value, clamped to ±PID_OUTPUT_LIMIT
 *   Positive correction -> steer left  (right wheel slower, left faster)
 *   Negative correction -> steer right (left  wheel slower, right faster)
 *------------------------------------------------------------------------------*/
static inline int PID_Update(PID_State *pid, int error){

    int p_term;
    int i_term;
    int d_term;
    int derivative;
    int output;

    /* Proportional */
    p_term = (PID_KP * error) / PID_GAIN_SCALE;

    /* Integral with anti-windup clamp */
    pid->integral += error;
    if(pid->integral >  PID_INTEGRAL_LIMIT){ pid->integral =  PID_INTEGRAL_LIMIT; }
    if(pid->integral < -PID_INTEGRAL_LIMIT){ pid->integral = -PID_INTEGRAL_LIMIT; }
    i_term = (PID_KI * pid->integral) / PID_GAIN_SCALE;

    /* Derivative */
    derivative    = error - pid->prev_error;
    d_term        = (PID_KD * derivative) / PID_GAIN_SCALE;
    pid->prev_error = error;

    /* Sum */
    output = p_term + i_term + d_term;

    /* Output clamp */
    if(output >  PID_OUTPUT_LIMIT){ output =  PID_OUTPUT_LIMIT; }
    if(output < -PID_OUTPUT_LIMIT){ output = -PID_OUTPUT_LIMIT; }

    return output;
}

#endif /* PID_H_ */
