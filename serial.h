/*------------------------------------------------------------------------------
 * File:        serial.h
 * Target:      MSP430FR2355
 *
 * Description: Header for eUSCI_A0 (IOT/J9) and eUSCI_A1 (PC/J14) UART.
 *
 *              Baud rate register derivation  (SMCLK = 8 MHz, OS16 = 1):
 *
 *              115,200 Baud:
 *                N      = 8,000,000 / 115,200 = 69.444
 *                UCBRx  = INT(N/16) = 4
 *                UCFx   = INT((N/16 - 4) * 16) = 5
 *                frac N = 0.444  ->  UCBRSx = 0x55  (Table 18-4 row 0.4378)
 *                MCTLW  = (0x55<<8)|(5<<4)|1 = 0x5551
 *
 *              460,800 Baud:
 *                N      = 8,000,000 / 460,800 = 17.361
 *                UCBRx  = INT(N/16) = 1
 *                UCFx   = INT((N/16 - 1) * 16) = 1
 *                frac N = 0.361  ->  UCBRSx = 0x4a
 *                MCTLW  = (0x4A<<8)|(1<<4)|1 = 0x4A11
 *------------------------------------------------------------------------------*/

#ifndef SERIAL_H_
#define SERIAL_H_

#include "msp430.h"

/*------------------------------------------------------------------------------
 * Ring buffer size
 *------------------------------------------------------------------------------*/
#define SERIAL_RING_SIZE        (64u)
#define SERIAL_BEGINNING        (0u)

/*------------------------------------------------------------------------------
 * Baud selectors
 *------------------------------------------------------------------------------*/
#define BAUD_115200             (0u)
#define BAUD_460800             (1u)

/*------------------------------------------------------------------------------
 * Baud register values
 *------------------------------------------------------------------------------*/
#define BR_115200_UCBRx         (4u)
#define BR_115200_MCTLW         (0x5551u)   /* UCBRSx=0x55, UCFx=5, UCOS16=1 */

#define BR_460800_UCBRx         (1u)
#define BR_460800_MCTLW         (0x4A11u)   /* UCBRSx=0x4A, UCFx=1, UCOS16=1 */

/*------------------------------------------------------------------------------
 * Transmit array  –  "NCSU  #1"  (two spaces between U and #)
 *------------------------------------------------------------------------------*/
#define NCSU_STRING             "NCSU  #1"
#define NCSU_STRING_LEN         (8u)

/*------------------------------------------------------------------------------
 * Timing (200 ms ticks)
 *------------------------------------------------------------------------------*/
#define SPLASH_TICKS            (25u)   /* 5 s splash                         */
#define POST_BAUD_CHANGE_TICKS  (10u)   /* 2 s wait before transmit           */

/*------------------------------------------------------------------------------
 * Receive display buffer length (chars shown on LCD line 0)
 * LCD line is 10 chars wide; we use all 10 positions.
 *------------------------------------------------------------------------------*/
#define RX_DISPLAY_LEN          (10u)

/*------------------------------------------------------------------------------
 * Externs (defined in serial.c)
 *------------------------------------------------------------------------------*/
extern volatile unsigned int  uca0_rx_wr;
extern volatile unsigned int  uca0_rx_rd;
extern volatile char          UCA0_Char_Rx[SERIAL_RING_SIZE];

extern volatile unsigned int  uca1_rx_wr;
extern volatile unsigned int  uca1_rx_rd;
extern volatile char          UCA1_Char_Rx[SERIAL_RING_SIZE];

/*
 * rx_display_buf  – 10-char rolling window of received characters shown on
 *                   LCD line 0.  Written by ISR, read by main loop.
 * rx_display_updated – set TRUE by ISR each time a new char arrives so the
 *                      main loop knows to refresh the LCD.
 */
extern volatile char          rx_display_buf[RX_DISPLAY_LEN + 1u];
extern volatile unsigned char rx_display_updated;

extern volatile unsigned char current_baud;

/*------------------------------------------------------------------------------
 * Function prototypes
 *------------------------------------------------------------------------------*/
void Init_Serial_UCA0(unsigned char baud);
void Init_Serial_UCA1(unsigned char baud);
void Set_Baud_UCA0(unsigned char baud);
void Set_Baud_UCA1(unsigned char baud);
void UCA1_Transmit_String(const char *str, unsigned int len);

#endif /* SERIAL_H_ */
