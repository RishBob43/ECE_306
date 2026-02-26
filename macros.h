#ifndef MACROS_H_
#define MACROS_H_

#define ALWAYS                  (1)
#define RESET_STATE             (0)
#define RED_LED              (0x01)
#define GRN_LED              (0x40)
#define TEST_PROBE           (0x01)
#define TRUE                 (0x01)
#define FALSE                (0x00)

#define USE_GPIO             (0x00)
#define USE_SMCLK            (0x01)

//------------------------------------------------------------------------------
// Timer B0 - 1ms tick at SMCLK = 8MHz
//
// SMCLK = 8,000,000 Hz
// ID__8 = /8  -> 1,000,000 Hz timer clock
// CCR0  = 1000 -> 1,000,000 / 1000 = 1,000 Hz (exactly 1ms)
//------------------------------------------------------------------------------
#define TB0_1MS_COUNT        (1000)  // CCR0 reload value -> exactly 1ms tick

#define DISPLAY_UPDATE_MS    (100)
#define TIME_SEQ_MS          (50)
#define TIME_SEQ_MAX         (250)

//------------------------------------------------------------------------------
// MCLK = 8MHz -> 8 cycles per microsecond
//------------------------------------------------------------------------------
#define CYCLES_PER_USEC      (8)
#define TEN_USEC             (10)

#endif /* MACROS_H_ */
