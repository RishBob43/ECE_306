#ifndef MACROS_H_
#define MACROS_H_

#define ALWAYS                  (1)
#define RESET_STATE             (0)
#define RED_LED              (0x01) // RED LED 0
#define GRN_LED              (0x40) // GREEN LED 1
#define TEST_PROBE           (0x01) // 3.0 TEST PROBE
#define TRUE                 (0x01)
#define FALSE                (0x00)

// Shape definitions
#define SHAPE_NONE           (0x00)
#define SHAPE_CIRCLE         (0x01)
#define SHAPE_FIGURE8        (0x02)
#define SHAPE_TRIANGLE       (0x03)

// Port 3 P3.4 mode selection
#define USE_GPIO             (0x00) // P3.4 configured as General Purpose I/O
#define USE_SMCLK            (0x01) // P3.4 configured as SMCLK output

//------------------------------------------------------------------------------
// Timer B0 - 1ms tick
//
// SMCLK = 500 kHz (8 MHz DCO / DIVM__2 / DIVS__8, set in clocks.c)
// Timer B0 input prescaler: ID__8  -> 500,000 / 8 = 62,500 Hz
// TB0_1MS_COUNT = 62            -> 62,500 / 62   ~= 1,008 Hz (~0.99ms per tick)
//
// NOTE: 62,500 / 1000 = 62.5 (not integer). CCR0=62 gives 1,008 Hz (~0.99ms).
// This is a ~0.8% timing error acceptable for display/shape timing purposes.
// TBIDEX is NOT used.
//------------------------------------------------------------------------------
#define TB0_1MS_COUNT        (62)   // CCR0 reload value  -> ~1ms tick

// How often (in 1ms ISR ticks) to set update_display = TRUE
#define DISPLAY_UPDATE_MS    (100)  // 100 ticks = 100ms refresh rate

// Time_Sequence is incremented once every TIME_SEQ_MS ticks (ms)
#define TIME_SEQ_MS          (50)   // 50ms per Time_Sequence step

// Time_Sequence rolls back to 0 when it reaches this value
// Matches the highest case in led.c (case 250)
#define TIME_SEQ_MAX         (250)

//------------------------------------------------------------------------------
// usleep / usleep10 timing constants
//
// MCLK = 8 MHz  ->  1 microsecond = 8 CPU cycles
// usleep(n)   delays n microseconds using __delay_cycles(CYCLES_PER_USEC)
// usleep10(n) delays n*10 microseconds (calls usleep(n * TEN_USEC))
//------------------------------------------------------------------------------
#define CYCLES_PER_USEC      (4)    // __delay_cycles count per 1 us (MCLK=4MHz)
#define TEN_USEC             (10)   // multiplier: usleep10(1) = 10 us

#endif /* MACROS_H_ */
