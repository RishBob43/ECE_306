#ifndef MACROS_H_
#define MACROS_H_

#define ALWAYS                  (1)
#define RESET_STATE             (0)
#define RED_LED              (0x01) // RED LED 0
#define GRN_LED              (0x40) // GREEN LED 1
#define TEST_PROBE           (0x01) // 3.0 TEST PROBE
#define TRUE                 (0x01)
#define FALSE                (0x00)

// Port 3 P3.4 mode selection
#define USE_GPIO             (0x00) // P3.4 configured as General Purpose I/O
#define USE_SMCLK            (0x01) // P3.4 configured as SMCLK output (500 kHz)

//------------------------------------------------------------------------------
// Timer B0 - 1ms tick
//
// SMCLK = 500 kHz  (8 MHz DCO / DIVS__16, set in clocks.c)
// Timer B0 input prescaler: ID__4 -> 500,000 / 4 = 125,000 Hz
// TB0_1MS_COUNT = 125          -> 125,000 / 125  = 1,000 Hz (exactly 1ms)
//------------------------------------------------------------------------------
#define TB0_1MS_COUNT        (125)  // CCR0 reload value -> exactly 1ms tick

// How often (in 1ms ISR ticks) to set update_display = TRUE
#define DISPLAY_UPDATE_MS    (100)  // 100 ticks = 100ms refresh rate

// Time_Sequence is incremented once every TIME_SEQ_MS ticks (ms)
#define TIME_SEQ_MS          (50)   // 50ms per Time_Sequence step

// Time_Sequence rolls back to 0 when it reaches this value
#define TIME_SEQ_MAX         (250)

//------------------------------------------------------------------------------
// usleep / usleep10 timing constants
//
// MCLK = 8 MHz  ->  1 microsecond = 8 CPU cycles
//------------------------------------------------------------------------------
#define CYCLES_PER_USEC  (4)   // MCLK = 4MHz -> 4 cycles per microsecond
#define TEN_USEC             (10)   // multiplier: usleep10(1) = 10 us

#endif /* MACROS_H_ */
