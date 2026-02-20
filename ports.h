#define FALSE                  (0x00)
#define TRUE                   (0x01)
#define MOTOR                  (0x00)
#define SMCLK_OFF              (0x00)
#define SMCLK_ON               (0x01)
#define PORTS                  (0x00)
#define PWM_MODE               (0x01)
#define WHEEL_OFF              (0x00)
#define WHEEL_PERIOD          (10000)
#define RIGHT_FORWARD_SPEED (TB3CCR2)
#define LEFT_FORWARD_SPEED  (TB3CCR3)
#define RIGHT_REVERSE_SPEED (TB3CCR4)
#define LEFT_REVERSE_SPEED  (TB3CCR5)
#define STEP                   (2000)
#define FORWARD                (0x00)
#define REVERSE                (0x01)

// Port 1 Pins
#define RED_LED                (0x01) // 1.0 RED LED
#define V_A1_SEEED             (0x02) // 1.1
#define V_DETECT_L             (0x04) // 1.2
#define V_DETECT_R             (0x08) // 1.3
#define V_A4_SEEED             (0x10) // 1.4
#define V_THUMB                (0x20) // 1.5
#define UCA0RXD                (0x40) // 1.6
#define UCA0TXD                (0x80) // 1.7

// Port 2 Pins
#define SLOW_CLK               (0x01) // 2.0
#define CHECK_BAT              (0x02) // 2.1
#define IR_LED                 (0x04) // 2.2
#define SW2                    (0x08) // 2.3
#define IOT_RUN_RED            (0x10) // 2.4
#define DAC_ENB                (0x20) // 2.5
#define LFXOUT                 (0x40) // 2.6
#define LFXIN                  (0x80) // 2.7

// Port 3 Pins
#define TEST_PROBE             (0x01) // 3.0
#define OA2O                   (0x02) // 3.1
#define OA2N                   (0x04) // 3.2
#define OA2P                   (0x08) // 3.3
#define SMCLK                  (0x10) // 3.4 SMCLK output pin
#define DAC_CNTL               (0x20) // 3.5
#define IOT_LINK_CPU           (0x40) // 3.6
#define IOT_EN_CPU             (0x80) // 3.7

// Port 4 Pins
#define RESET_LCD              (0x01) // 4.0
#define SW1                    (0x02) // 4.1
#define UCA1RXD                (0x04) // 4.2
#define UCA1TXD                (0x08) // 4.3
#define UCB1_CS_LCD            (0x10) // 4.4
#define UCB1CLK                (0x20) // 4.5
#define UCB1SIMO               (0x40) // 4.6
#define UCB1SOMI               (0x80) // 4.7

// Port 5 Pins
#define V_BAT                  (0x01) // 5.0
#define V_5                    (0x02) // 5.1
#define V_DAC                  (0x04) // 5.2
#define V_3_3                  (0x08) // 5.3
#define IOT_BOOT_CPU           (0x10) // 5.4

// Port 6 Pins
#define LCD_BACKLITE           (0x01) // 6.0
#define R_FORWARD              (0x02) // 6.1
#define L_FORWARD              (0x04) // 6.2
#define R_REVERSE              (0x08) // 6.3
#define L_REVERSE              (0x10) // 6.4
#define P6_5                   (0x20) // 6.5
#define GRN_LED                (0x40) // 6.6

void Init_Ports(void);
