#ifndef MACROS_H_
#define MACROS_H_

#define ALWAYS                  (1)
#define RESET_STATE             (0)
#define RED_LED              (0x01) // RED LED 0
#define GRN_LED              (0x40) // GREEN LED 1
#define TEST_PROBE           (0x01) // 0 TEST PROBE
#define TRUE                 (0x01) //
#define FALSE                (0x00) //

// Shape definitions
#define SHAPE_NONE           (0x00)
#define SHAPE_CIRCLE         (0x01)
#define SHAPE_FIGURE8        (0x02)
#define SHAPE_TRIANGLE       (0x03)

// Port 3 SMCLK configuration
#define USE_GPIO             (0x00)
#define USE_SMCLK            (0x01)

// Clock configuration
#define DCO_MULT_8MHZ         (243)   // FLL multiplier for 8MHz: (8MHz/REFO 32768Hz) - 1
#define FLL_SETTLE_CYCLES       (3)   // Minimum cycles to wait after enabling FLL
#define SMCLK_DIV_MASK       (0x0007) // Mask for DIVS bits in CSCTL5
#define SMCLK_DIV_500KHZ     (0x0004) // DIVS = 100b -> 8MHz / 16 = 500kHz
#define SMCLK_DIV_1MHZ       (0x0003) // DIVS = 011b -> 8MHz / 8  = 1MHz
#define SMCLK_DIV_8MHZ       (0x0000) // DIVS = 000b -> 8MHz / 1  = 8MHz

// Timer B0 configuration (1ms tick from 8MHz SMCLK)
// SMCLK / ID__8 / TBIDEX__8 = 8MHz / 8 / 8 = 125kHz
// 125kHz / 1000 Hz = 125 counts per 1ms
#define TB0_1MS_COUNT         (125)   // CCR0 count for 1ms interrupt
#define TB0_HALF_COUNT         (62)   // CCR1/CCR2 half-period value

// Timer B0 ISR timing thresholds
#define DISPLAY_UPDATE_MS     (100)   // Update display every 100ms
#define TIME_SEQ_MS            (50)   // Increment Time_Sequence every 50ms
#define TIME_SEQ_MAX          (250)   // Time_Sequence rollover value

// Timer start value
#define TIMER_START             (1)   // Initial value when starting shape_timer

// usleep cycles per microsecond (8MHz -> 8 cycles = 1us)
#define CYCLES_PER_USEC         (8)

// Five-millisecond multiplier
#define FIVE_MS                 (5)

// Ten-microsecond multiplier
#define TEN_USEC               (10)

#endif /* MACROS_H_ */
