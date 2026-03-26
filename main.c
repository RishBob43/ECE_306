/*------------------------------------------------------------------------------
 * File:        main.c  (Project 7 – Bang-Bang + DAC)
 * Target:      MSP430FR2355
 *
 * Description: Black-line circle follower with:
 *   - Two-phase calibration (white then black, emitter ON throughout)
 *   - Dynamic threshold = midpoint of white/black averages
 *   - Modified bang-bang steering controller (4-case + last-direction memory)
 *   - DAC-controlled motor supply voltage via LT1935 buck-boost (~4.3 V)
 *   - TB3 PWM differential steering
 *   - Two-lap counter using right-detector rising-edge detection
 *   - Automatic inward exit to circle center
 *
 * Bang-bang controller cases:
 *   BOTH on line   -> drive straight, both wheels at BASE_SPEED
 *   LEFT off line  -> steer right: slow left wheel, speed right wheel
 *   RIGHT off line -> steer left:  slow right wheel, speed left wheel
 *   BOTH off line  -> use g_last_correction to repeat last known good turn
 *
 * Tick handling (dropped-tick fix):
 *   The CCR0 ISR increments update_display_count each 200 ms instead of
 *   setting a simple boolean.  The main loop consumes ONE count per pass and
 *   sets tick_200ms = TRUE for that pass.  If the loop runs slow (LCD, ADC)
 *   the count queues up and the next pass catches up immediately – no ticks
 *   are silently dropped.
 *
 * State machine:
 *   CALIBRATE -> IDLE -> DELAY -> INTERCEPT -> WAIT -> ALIGN
 *             -> CIRCLE -> EXIT -> STOPPED
 *
 * Display (4 lines x 10 chars, every 200 ms):
 *   Line 0: state label     "CIRCLING  "
 *   Line 1: right detector  "R:0742    "
 *   Line 2: left  detector  "L:0318    "
 *   Line 3: elapsed time    "T: 12.4s  "
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "timerB0.h"
#include "ADC.h"
#include "hex_to_bcd.h"
#include "dac.h"

/*==============================================================================
 * Externs
 *============================================================================*/
extern char                   display_line[4][11];
extern char                  *display[4];
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char update_display;
extern volatile unsigned int  update_display_count;
extern volatile unsigned char display_changed;

/*==============================================================================
 * Local symbolic constants
 *
 * NOTE: STATE_*, TICKS_*, LAP_EDGE_COUNT, TICKS_EXIT_DRIVE,
 *       TICKS_ALIGN_TIMEOUT, TICKS_CAL_SETTLE, NUM_CAL_SAMPLES, and
 *       CAL_THRESHOLD_MARGIN are defined in macros.h.
 *       PID_BASE_SPEED, PID_MIN_SPEED, PID_MAX_SPEED are in dac.h.
 *       None are redefined here to avoid #48-D redefinition warnings.
 *============================================================================*/

/*-- GPIO straight drive (INTERCEPT and CIRCLE straight segments) ------------*/
/*  Both forward pins HIGH via P6OUT.  No PWM – DAC sets the speed.          */
#define MOTORS_GPIO_FORWARD()    do { P6OUT |=  (R_FORWARD | L_FORWARD);      \
                                      P6OUT &= ~(R_REVERSE  | L_REVERSE);     } while(0)

/*-- GPIO stop ----------------------------------------------------------------*/
#define MOTORS_STOP_GPIO()       do { P6OUT &= ~(R_FORWARD | L_FORWARD |      \
                                                  R_REVERSE | L_REVERSE);     } while(0)

/*-- PWM stop (zero all CCR duty cycles) -------------------------------------*/
#define MOTORS_STOP_PWM()        do { TB3CCR2 = WHEEL_OFF;                    \
                                      TB3CCR3 = WHEEL_OFF;                    \
                                      TB3CCR4 = WHEEL_OFF;                    \
                                      TB3CCR5 = WHEEL_OFF; } while(0)

/*-- Combined full stop (both GPIO and PWM) ----------------------------------*/
#define MOTORS_STOP_ALL()        do { MOTORS_STOP_GPIO(); MOTORS_STOP_PWM(); } while(0)

/*-- GPIO alignment turn: left wheel reverse, right forward ------------------*/
/*  Pivots car left so right sensor sweeps onto the line.                    */
#define MOTOR_TURN_LEFT_GPIO()   do { P6OUT |=  (R_FORWARD | L_REVERSE);      \
                                      P6OUT &= ~(L_FORWARD  | R_REVERSE);     } while(0)

/*-- GPIO inward exit turn: left forward, right reverse ----------------------*/
#define MOTOR_INWARD_TURN_GPIO() do { P6OUT |=  (L_FORWARD | R_REVERSE);      \
                                      P6OUT &= ~(R_FORWARD  | L_REVERSE);     } while(0)

/*-- Display line indices ----------------------------------------------------*/
#define DISP_STATE_LINE         (0u)
#define DISP_RIGHT_LINE         (1u)
#define DISP_LEFT_LINE          (2u)
#define DISP_TIMER_LINE         (3u)

/*-- Bang-bang last-correction direction codes --------------------------------*/
/*  Stored in g_last_correction so the BOTH-off-line case knows which way    */
/*  to keep turning.                                                          */
#define CORRECTION_NONE         (0x00)   /* No correction applied yet         */
#define CORRECTION_LEFT         (0x01)   /* Last correction steered left      */
#define CORRECTION_RIGHT        (0x02)   /* Last correction steered right     */

/*==============================================================================
 * Module globals
 *============================================================================*/
static unsigned int  g_white_left    = RESET_STATE;
static unsigned int  g_white_right   = RESET_STATE;
static unsigned int  g_black_left    = RESET_STATE;
static unsigned int  g_black_right   = RESET_STATE;
static unsigned int  g_threshold     = BLACK_LINE_THRESHOLD;

static volatile unsigned int  g_timer_ticks  = RESET_STATE;
static volatile unsigned char g_timer_active = FALSE;

static unsigned char g_lap_edges        = RESET_STATE;
static unsigned char g_prev_right_black = FALSE;
static unsigned char g_exit_phase       = RESET_STATE;   /* 0=turning, 1=straight */

/*-- Bang-bang direction memory -----------------------------------------------*/
/*  Updated every time the LEFT-off or RIGHT-off case fires.  Used by the    */
/*  BOTH-off case to maintain the last known good correction direction.       */
static unsigned char g_last_correction  = CORRECTION_NONE;

/*==============================================================================
 * Forward declarations
 *============================================================================*/
static void          set_line(int line, const char *msg);
static void          show_detector_values(void);
static void          show_timer(void);
static void          flush_display(void);
static unsigned int  average_adc_samples(unsigned char channel_flag);
static void          run_calibration(void);
static void          update_dynamic_threshold(void);
static unsigned char is_black_left_dyn(void);
static unsigned char is_black_right_dyn(void);
static unsigned char get_line_state_dyn(void);
static void          line_follow_bangbang(void);

/*==============================================================================
 * set_line – copy 10-char literal into display buffer and flag changed
 *==============================================================================*/
static void set_line(int line, const char *msg){
    strncpy(display_line[line], msg, DISPLAY_LINE_LENGTH - 1u);
    display_line[line][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;
    display_changed = TRUE;
}

/*==============================================================================
 * flush_display – force an immediate LCD write
 *   Use only during blocking calibration phases where the main loop is stalled
 *   and Display_Process() is never reached.
 *==============================================================================*/
static void flush_display(void){
    display_changed = TRUE;
    Display_Update(0, 0, 0, 0);
    display_changed = FALSE;
}

/*==============================================================================
 * show_detector_values – write R: and L: BCD to lines 1 & 2
 *==============================================================================*/
static void show_detector_values(void){

    HEX_to_BCD(ADC_Right_Detect);
    display_line[DISP_RIGHT_LINE][0] = 'R';
    display_line[DISP_RIGHT_LINE][1] = ':';
    display_line[DISP_RIGHT_LINE][2] = thousands;
    display_line[DISP_RIGHT_LINE][3] = hundreds;
    display_line[DISP_RIGHT_LINE][4] = tens;
    display_line[DISP_RIGHT_LINE][5] = ones;
    display_line[DISP_RIGHT_LINE][6] = ' ';
    display_line[DISP_RIGHT_LINE][7] = ' ';
    display_line[DISP_RIGHT_LINE][8] = ' ';
    display_line[DISP_RIGHT_LINE][9] = ' ';
    display_line[DISP_RIGHT_LINE][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;

    HEX_to_BCD(ADC_Left_Detect);
    display_line[DISP_LEFT_LINE][0] = 'L';
    display_line[DISP_LEFT_LINE][1] = ':';
    display_line[DISP_LEFT_LINE][2] = thousands;
    display_line[DISP_LEFT_LINE][3] = hundreds;
    display_line[DISP_LEFT_LINE][4] = tens;
    display_line[DISP_LEFT_LINE][5] = ones;
    display_line[DISP_LEFT_LINE][6] = ' ';
    display_line[DISP_LEFT_LINE][7] = ' ';
    display_line[DISP_LEFT_LINE][8] = ' ';
    display_line[DISP_LEFT_LINE][9] = ' ';
    display_line[DISP_LEFT_LINE][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;

    display_changed = TRUE;
}

/*==============================================================================
 * show_timer – format g_timer_ticks as "T:xxx.xs " on line 3
 *   1 tick = 200 ms = 0.2 s -> tenths = ticks * 2, range 0.0 – 999.8 s
 *==============================================================================*/
static void show_timer(void){

    unsigned int  tenths;
    unsigned int  secs;
    unsigned char tenth_digit;
    unsigned int  s_hundreds;
    unsigned int  s_tens;
    unsigned int  s_ones;

    tenths = g_timer_ticks * 2u;
    if(tenths > 9998u){ tenths = 9998u; }

    secs        = tenths / 10u;
    tenth_digit = (unsigned char)(tenths % 10u);

    s_hundreds = RESET_STATE;
    s_tens     = RESET_STATE;
    s_ones     = secs;
    while(s_ones >= 100u){ s_ones -= 100u; s_hundreds++; }
    while(s_ones >= 10u) { s_ones -= 10u;  s_tens++;     }

    display_line[DISP_TIMER_LINE][0] = 'T';
    display_line[DISP_TIMER_LINE][1] = ':';
    display_line[DISP_TIMER_LINE][2] = (char)(s_hundreds + ASCII_OFFSET);
    display_line[DISP_TIMER_LINE][3] = (char)(s_tens     + ASCII_OFFSET);
    display_line[DISP_TIMER_LINE][4] = (char)(s_ones     + ASCII_OFFSET);
    display_line[DISP_TIMER_LINE][5] = '.';
    display_line[DISP_TIMER_LINE][6] = (char)(tenth_digit + ASCII_OFFSET);
    display_line[DISP_TIMER_LINE][7] = 's';
    display_line[DISP_TIMER_LINE][8] = ' ';
    display_line[DISP_TIMER_LINE][9] = ' ';
    display_line[DISP_TIMER_LINE][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;

    display_changed = TRUE;
}

/*==============================================================================
 * average_adc_samples
 *   Waits for ADC_updated, collects NUM_CAL_SAMPLES, returns integer average.
 *   channel_flag: 0 = left, non-zero = right
 *==============================================================================*/
static unsigned int average_adc_samples(unsigned char channel_flag){
    unsigned int  sum   = RESET_STATE;
    unsigned char count = RESET_STATE;
    while(count < NUM_CAL_SAMPLES){
        if(ADC_updated){
            ADC_updated = FALSE;
            sum += (channel_flag == RESET_STATE) ? ADC_Left_Detect
                                                 : ADC_Right_Detect;
            count++;
        }
    }
    return (sum / NUM_CAL_SAMPLES);
}

/*==============================================================================
 * run_calibration
 *
 * Two-phase sequence (emitter stays ON for both):
 *   Phase 1 – car on WHITE paper: sample baseline
 *   Move window (4 s countdown): user moves car onto black tape
 *   Phase 2 – car on BLACK tape: sample line value
 *
 * Threshold = midpoint of white_avg and black_avg.
 * flush_display() forces immediate LCD updates during the blocking countdown.
 *==============================================================================*/
static void run_calibration(void){

    unsigned char settle;
    settle = RESET_STATE;
       while(settle < 5u){
           if(update_display_count > 0u){
               update_display_count--;
               settle++;
           }
       }

    /*-- Phase 1: white ------------------------------------------------------*/
    IR_LED_control(IR_LED_ON);
    set_line(DISP_STATE_LINE, "CAL:WHITE ");
    show_detector_values();
    set_line(DISP_TIMER_LINE, "Sampling..");
    flush_display();

    settle = RESET_STATE;
    while(settle < TICKS_CAL_SETTLE){
        if(update_display_count > 0u){
            update_display_count--;
            settle++;
        }
    }
    g_white_left  = average_adc_samples(0u);
    g_white_right = average_adc_samples(1u);

    /*-- 4-second move window: user moves car from white to black tape -------*/
    set_line(DISP_STATE_LINE, "MOVE CAR  ");
    show_detector_values();
    set_line(DISP_TIMER_LINE, "Wait: 4s  ");
    flush_display();

    settle = RESET_STATE;
    while(settle < 20u){           /* 20 x 200 ms = 4 s                       */
        if(update_display_count > 0u){
            update_display_count--;
            settle++;
            if(settle == 5u) { set_line(DISP_TIMER_LINE, "Wait: 3s  "); flush_display(); }
            if(settle == 10u){ set_line(DISP_TIMER_LINE, "Wait: 2s  "); flush_display(); }
            if(settle == 15u){ set_line(DISP_TIMER_LINE, "Wait: 1s  "); flush_display(); }
            if(settle == 19u){ set_line(DISP_TIMER_LINE, "HOLD STILL"); flush_display(); }
        }
    }

    /*-- Phase 2: black ------------------------------------------------------*/
    set_line(DISP_STATE_LINE, "CAL:BLACK ");
    show_detector_values();
    set_line(DISP_TIMER_LINE, "Sampling..");
    flush_display();

    settle = RESET_STATE;
    while(settle < TICKS_CAL_SETTLE){
        if(update_display_count > 0u){
            update_display_count--;
            settle++;
        }
    }
    g_black_left  = average_adc_samples(0u);
    g_black_right = average_adc_samples(1u);

    update_dynamic_threshold();
}

/*==============================================================================
 * update_dynamic_threshold
 *   Threshold = midpoint between white average and black average.
 *   Falls back to static BLACK_LINE_THRESHOLD if the two readings are too
 *   close together (margin < CAL_THRESHOLD_MARGIN) to be reliable.
 *==============================================================================*/
static void update_dynamic_threshold(void){

    unsigned int white_avg = (g_white_left  + g_white_right) / 2u;
    unsigned int black_avg = (g_black_left  + g_black_right) / 2u;

    if(black_avg > (white_avg + CAL_THRESHOLD_MARGIN)){
        g_threshold = (white_avg + 9 * black_avg) / 10;
    } else {
        g_threshold = BLACK_LINE_THRESHOLD;   /* static fallback from macros.h */
    }
}

/*==============================================================================
 * Threshold-based detection helpers
 *==============================================================================*/
static unsigned char is_black_left_dyn(void){
    return (ADC_Left_Detect + 5  > g_threshold) ? TRUE : FALSE;
}
static unsigned char is_black_right_dyn(void){
    return (ADC_Right_Detect > g_threshold) ? TRUE : FALSE;
}
static unsigned char get_line_state_dyn(void){
    unsigned char s = LINE_NONE;
    if(is_black_left_dyn())  s |= LINE_LEFT;
    if(is_black_right_dyn()) s |= LINE_RIGHT;
    return s;
}

/*==============================================================================
 * line_follow_bangbang
 *
 * Modified bang-bang steering using TB3 PWM on the forward channels.
 * Four cases based on which detectors see the black line:
 *
 * Case 1  BOTH on line   -> straight ahead, both wheels at PID_BASE_SPEED.
 * g_last_correction is NOT updated.
 *
 * Case 2  LEFT off line  -> drifted right; steer left.
 * Left wheel slows to PID_MIN_SPEED,
 * Right wheel at PID_BASE_SPEED.
 * g_last_correction = CORRECTION_LEFT.
 *
 * Case 3  RIGHT off line -> drifted left; steer right.
 * Right wheel slows to PID_MIN_SPEED,
 * Left wheel at PID_BASE_SPEED.
 * g_last_correction = CORRECTION_RIGHT.
 *
 * Case 4  BOTH off line  -> completely lost; repeat last correction at
 * full authority.
 * If no correction has been applied yet
 * (CORRECTION_NONE) default to steering left,
 * which is the correct bias for a CCW circle.
 *==============================================================================*/
static void line_follow_bangbang(void){

    unsigned char left_black  = is_black_left_dyn();
    unsigned char right_black = is_black_right_dyn();

    /* Clear GPIO forward bits so PWM drives the H-bridge exclusively */
    P6OUT &= ~(R_FORWARD | L_FORWARD | R_REVERSE | L_REVERSE);

    /* Reverse channels always off during line following */
    TB3CCR4 = WHEEL_OFF;
    TB3CCR5 = WHEEL_OFF;

    if(left_black && right_black){
        /*----------------------------------------------------------------------
         * Case 1: BOTH sensors on black line -> go straight.
         *--------------------------------------------------------------------*/
        TB3CCR2 = 15000;    /* R_FORWARD */
        TB3CCR3 = 15000;    /* L_FORWARD */

    } else if(!left_black && right_black){
        /*----------------------------------------------------------------------
         * Case 2: LEFT sensor off line, RIGHT on line.
         * Car has drifted right of the line edge -> steer left.
         * Slow the left wheel to pivot toward the line.
         *--------------------------------------------------------------------*/
        RIGHT_REVERSE_SPEED = PID_MIN_SPEED;     /* R_FORWARD – slow right wheel */
        TB3CCR3 = PID_BASE_SPEED;    /* L_FORWARD – full left wheel  */
        g_last_correction = CORRECTION_RIGHT;

    } else if(left_black && !right_black){
        /*----------------------------------------------------------------------
         * Case 3: RIGHT sensor off line, LEFT on line.
         * Car has drifted left of the line edge -> steer right.
         * Slow the right wheel to pivot toward the line.
         *--------------------------------------------------------------------*/
        TB3CCR2 = PID_BASE_SPEED;    /* R_FORWARD – full right wheel */
        LEFT_REVERSE_SPEED = PID_MIN_SPEED;     /* L_FORWARD – slow left wheel  */
        g_last_correction = CORRECTION_LEFT;

    }
     else {
        /*----------------------------------------------------------------------
         * Case 4: BOTH sensors off line -> completely lost.
         * Match the last correction direction at full authority.
         *--------------------------------------------------------------------*/
        if(g_last_correction == CORRECTION_RIGHT){
            /* steering right */
            RIGHT_REVERSE_SPEED = PID_MIN_SPEED;     /* R_FORWARD – slow right wheel */
            TB3CCR3 = PID_BASE_SPEED;    /* L_FORWARD – full left wheel  */
        } else {
            /* CORRECTION_LEFT or CORRECTION_NONE -> steer left */
            TB3CCR2 = PID_BASE_SPEED;    /* R_FORWARD – full right wheel */
            LEFT_REVERSE_SPEED = PID_MIN_SPEED;     /* L_FORWARD – slow left wheel  */
        }
    }
}

/*==============================================================================
 * main
 *==============================================================================*/
void main(void){

    unsigned char state       = STATE_CALIBRATE;
    unsigned int  state_timer = RESET_STATE;
    unsigned char tick_200ms  = FALSE;   /* TRUE for exactly one loop pass per tick */

    PM5CTL0 &= ~LOCKLPM5;

    Init_Ports();
    Init_Clocks();
    Init_Conditions();
    Init_Timers();
    Init_LCD();
    Init_ADC();
    Init_Switches();

    MOTORS_STOP_ALL();
    IR_LED_control(IR_LED_ON);
    g_last_correction = CORRECTION_NONE;

    /*==========================================================================
     * Main loop
     *========================================================================*/
    while(ALWAYS){

        /*----------------------------------------------------------------------
         * Tick consumption - drain ONE queued 200 ms tick per loop pass.
         *
         * The CCR0 ISR increments update_display_count every 200 ms.
         * We consume one count here and set update_display = TRUE so that
         * Display_Process() (called immediately after) actually triggers an
         * LCD write.
         *
         * tick_200ms gates state_timer increments so timers only advance on
         * real 200 ms boundaries rather than every CPU cycle.
         *--------------------------------------------------------------------*/
        tick_200ms = FALSE;
        if(update_display_count > 0u){
            update_display_count--;
            update_display = TRUE;       /* tell Display_Process to write LCD  */
            tick_200ms     = TRUE;

            /* Advance elapsed timer while car is moving */
            if(g_timer_active){
                if(g_timer_ticks < 4999u){ g_timer_ticks++; }
            }

            /* Refresh detector + timer display during active motion states */
            if((state == STATE_INTERCEPT) ||
               (state == STATE_CIRCLE)    ||
               (state == STATE_EXIT)){
                show_detector_values();
                show_timer();
            }
        }

        /*----------------------------------------------------------------------
         * Display_Process is called AFTER the tick block so it sees
         * update_display = TRUE and writes the LCD this pass.
         * It clears update_display and display_changed internally.
         *--------------------------------------------------------------------*/
        Display_Process();

        /*----------------------------------------------------------------------
         * State machine
         *--------------------------------------------------------------------*/
        switch(state){

            /*-- CALIBRATE ---------------------------------------------------*/
            case STATE_CALIBRATE:
                run_calibration();

                /* Display computed threshold on line 3 */
                HEX_to_BCD(g_threshold);
                display_line[DISP_TIMER_LINE][0] = 'T';
                display_line[DISP_TIMER_LINE][1] = 'H';
                display_line[DISP_TIMER_LINE][2] = ':';
                display_line[DISP_TIMER_LINE][3] = thousands;
                display_line[DISP_TIMER_LINE][4] = hundreds;
                display_line[DISP_TIMER_LINE][5] = tens;
                display_line[DISP_TIMER_LINE][6] = ones;
                display_line[DISP_TIMER_LINE][7] = ' ';
                display_line[DISP_TIMER_LINE][8] = ' ';
                display_line[DISP_TIMER_LINE][9] = ' ';
                display_line[DISP_TIMER_LINE][DISPLAY_LINE_LENGTH-1u] = RESET_STATE;

                set_line(DISP_STATE_LINE, "CAL DONE  ");
                show_detector_values();
                flush_display();

                Init_DAC();   /* Start DAC soft-start ramp while user reads LCD */

                /*--------------------------------------------------------------
                 * Discard any SW1 press that occurred during the blocking
                 * calibration countdown so the car does not auto-launch.
                 *------------------------------------------------------------*/
                sw1_pressed = FALSE;

                state = STATE_IDLE;
                break;

            /*-- IDLE --------------------------------------------------------*/
            case STATE_IDLE:
                if(tick_200ms){
                    show_detector_values();

                    if(DAC_ready == FALSE){
                        set_line(DISP_STATE_LINE, "DAC RAMP  ");
                        set_line(DISP_TIMER_LINE, "Wait...   ");
                        sw1_pressed = FALSE;
                    } else {
                        set_line(DISP_STATE_LINE, "SW1 GO!   ");
                        HEX_to_BCD(g_threshold);
                        display_line[DISP_TIMER_LINE][0] = 'G';
                        display_line[DISP_TIMER_LINE][1] = ':';
                        display_line[DISP_TIMER_LINE][2] = thousands;
                        display_line[DISP_TIMER_LINE][3] = hundreds;
                        display_line[DISP_TIMER_LINE][4] = tens;
                        display_line[DISP_TIMER_LINE][5] = ones;
                        display_line[DISP_TIMER_LINE][6] = ' ';
                        display_line[DISP_TIMER_LINE][7] = ' ';
                        display_line[DISP_TIMER_LINE][8] = ' ';
                        display_line[DISP_TIMER_LINE][9] = ' ';
                        display_line[DISP_TIMER_LINE][DISPLAY_LINE_LENGTH-1u] = RESET_STATE;
                        display_changed = TRUE;
                    }
                }

                if(sw1_pressed && (DAC_ready == TRUE)){
                    sw1_pressed       = FALSE;
                    g_timer_ticks     = RESET_STATE;
                    g_timer_active    = FALSE;
                    state_timer       = RESET_STATE;
                    g_last_correction = CORRECTION_NONE;   /* reset bang-bang memory */
                    state = STATE_DELAY;
                    set_line(DISP_STATE_LINE, "WAITING   ");
                    set_line(DISP_TIMER_LINE, "T:  0.0s  ");
                }
                break;

            /*-- DELAY: 1 s pause then start PWM forward ---------------------*/
            case STATE_DELAY:
                if(tick_200ms){ state_timer++; }
                if(state_timer >= TICKS_1_SEC){
                    state_timer    = RESET_STATE;
                    g_timer_active = TRUE;
                    IR_LED_control(IR_LED_ON);
                    DAC_Set(DAC_CRUISE);        /* controlled speed            */
                    TB3CCR2 = PID_BASE_SPEED;   /* R_FORWARD                  */
                    TB3CCR3 = PID_BASE_SPEED;   /* L_FORWARD                  */
                    TB3CCR4 = WHEEL_OFF;
                    TB3CCR5 = WHEEL_OFF;
                    state = STATE_INTERCEPT;
                    set_line(DISP_STATE_LINE, "INTERCEPT ");
                }
                break;

            /*-- INTERCEPT: drive forward until line detected ----------------*/
            case STATE_INTERCEPT:
                if(get_line_state_dyn() != LINE_NONE){
                    MOTORS_STOP_ALL();
                    state_timer = RESET_STATE;
                    state       = STATE_WAIT;
                    set_line(DISP_STATE_LINE, "WAITING   ");
                }
                break;

            /*-- WAIT: hold 4 s, then begin alignment turn -------------------*/
            case STATE_WAIT:
                if(tick_200ms){ state_timer++; }
                if(state_timer >= TICKS_4_SEC){
                    state_timer = RESET_STATE;
                    state       = STATE_ALIGN;
                    /* Left pivot turn: right forward, left reverse */
                    TB3CCR2 = WHEEL_OFF;
                    TB3CCR3 = WHEEL_OFF;
                    TB3CCR4 = WHEEL_OFF;
                    TB3CCR5 = PID_MIN_SPEED;   /* L_REVERSE */
                    set_line(DISP_STATE_LINE, "TURNING   ");
                }
                break;

            /*-- ALIGN: pivot left until both sensors on black ---------------*/
            case STATE_ALIGN:
                if(get_line_state_dyn() == LINE_BOTH){
                    MOTORS_STOP_GPIO();
                    g_last_correction  = CORRECTION_NONE;   /* fresh start    */
                    g_lap_edges        = RESET_STATE;
                    g_prev_right_black = is_black_right_dyn();
                    state_timer        = RESET_STATE;
                    state              = STATE_CIRCLE;
                    set_line(DISP_STATE_LINE, "CIRCLING  ");
                    line_follow_bangbang();     /* First bang-bang step        */
                } else {
                    if(tick_200ms){ state_timer++; }
                    if(state_timer >= TICKS_ALIGN_TIMEOUT){
                        MOTORS_STOP_ALL();
                        g_timer_active = FALSE;
                        state = STATE_STOPPED;
                        set_line(DISP_STATE_LINE, "STOPPED   ");
                    }
                }
                break;

            /*-- CIRCLE: bang-bang steering, count 2 laps --------------------*/
            case STATE_CIRCLE:
                if(tick_200ms){ state_timer++; }
                line_follow_bangbang();
                if(state_timer >= 120){
                    MOTORS_STOP_ALL();
                    state       = STATE_EXIT;
                    state_timer  = RESET_STATE;
                    TB3CCR2 = 30000;
                    TB3CCR3 = WHEEL_OFF;
                    TB3CCR4 = WHEEL_OFF;
                    TB3CCR5 = 30000;
                    set_line(DISP_STATE_LINE, "EXITING   ");
               }
                break;

            /*-- EXIT: inward turn then drive to center ----------------------*/
            case STATE_EXIT:
                if(tick_200ms){ state_timer++; }

                if(g_exit_phase == 0u){
                    if(state_timer >= TICKS_TURN){
                        state_timer  = RESET_STATE;
                        g_exit_phase = 1u;
                        TB3CCR2 = PID_BASE_SPEED;   /* R_FORWARD */
                        TB3CCR3 = PID_BASE_SPEED;   /* L_FORWARD */
                        TB3CCR4 = WHEEL_OFF;
                        TB3CCR5 = WHEEL_OFF;
                    }
                } else {
                    if(state_timer >= TICKS_EXIT_DRIVE){
                        MOTORS_STOP_ALL();
                        g_timer_active = FALSE;
                        g_exit_phase   = RESET_STATE;
                        state = STATE_STOPPED;
                        set_line(DISP_STATE_LINE, "STOPPED   ");
                        show_detector_values();
                        show_timer();
                    }
                }
                break;

            /*-- STOPPED: freeze display, cut power, await SW1 ---------------*/
            case STATE_STOPPED:
                MOTORS_STOP_ALL();
                IR_LED_control(IR_LED_OFF);
                P2OUT &= ~DAC_ENB;       /* Cut LT1935 – save battery          */

                if(tick_200ms){ show_detector_values(); }

                if(sw1_pressed){
                    sw1_pressed = FALSE;
                    P2OUT |= DAC_ENB;
                    state = STATE_CALIBRATE;
                }
                break;

            default:
                MOTORS_STOP_ALL();
                state = STATE_IDLE;
                break;
        }

    } /* while(ALWAYS) */
}
