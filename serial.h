/*------------------------------------------------------------------------------
 * File:        serial.h
 * Target:      MSP430FR2355
 *
 * Description: eUSCI_A0 (IOT/J9) and eUSCI_A1 (PC/J14) UART header.
 *
 *              Project 9 additions:
 *                - IOT_2_PC / PC_2_IOT ring buffers for bidirectional bridge
 *                - pc_tx_enabled gate (suppress TX to PC until first RX char)
 *                - ^ command parser state and buffer
 *                - IOT reset helper
 *
 *              Baud rate register derivation  (SMCLK = 8 MHz, OS16 = 1):
 *
 *              115,200 Baud:
 *                N      = 8,000,000 / 115,200 = 69.444
 *                UCBRx  = INT(N/16) = 4
 *                UCFx   = INT((N/16 - 4) * 16) = 5
 *                UCBRSx = 0x55  (Table 18-4)
 *                MCTLW  = 0x5551
 *
 *              460,800 Baud:
 *                N      = 8,000,000 / 460,800 = 17.361
 *                UCBRx  = INT(N/16) = 1
 *                UCFx   = 1
 *                UCBRSx = 0x4A
 *                MCTLW  = 0x4A11
 *
 *              9,600 Baud:
 *                N      = 8,000,000 / 9,600 = 833.333
 *                UCBRx  = INT(N/16) = 52
 *                UCFx   = 1
 *                UCBRSx = 0x49
 *                MCTLW  = 0x4911
 *------------------------------------------------------------------------------*/

#ifndef SERIAL_H_
#define SERIAL_H_

#include "msp430.h"

/*------------------------------------------------------------------------------
 * Ring buffer sizes
 *------------------------------------------------------------------------------*/
#define SERIAL_RING_SIZE        (64u)
#define SMALL_RING_SIZE         (128u)
#define SERIAL_BEGINNING        (0u)
#define BEGINNING               (0u)

/*------------------------------------------------------------------------------
 * Baud selectors
 *------------------------------------------------------------------------------*/
#define BAUD_9600               (2u)
#define BAUD_115200             (0u)
#define BAUD_460800             (1u)

/*------------------------------------------------------------------------------
 * Baud register values
 *------------------------------------------------------------------------------*/
#define BR_115200_UCBRx         (4u)
#define BR_115200_MCTLW         (0x5551u)

#define BR_460800_UCBRx         (1u)
#define BR_460800_MCTLW         (0x4A11u)

#define BR_9600_UCBRx           (52u)
#define BR_9600_MCTLW           (0x4911u)

/*------------------------------------------------------------------------------
 * Transmit strings
 *------------------------------------------------------------------------------*/
#define NCSU_STRING             "NCSU  #1"
#define NCSU_STRING_LEN         (8u)

/*------------------------------------------------------------------------------
 * Timing (200 ms ticks)
 *------------------------------------------------------------------------------*/
#define SPLASH_TICKS            (25u)
#define POST_BAUD_CHANGE_TICKS  (10u)

/*------------------------------------------------------------------------------
 * Receive display buffer
 *------------------------------------------------------------------------------*/
#define RX_DISPLAY_LEN          (10u)

/*------------------------------------------------------------------------------
 * IOT reset timing
 *   IOT_RESET_MS: P3.7 held LOW (100 ms minimum per spec)
 *   IOT_SETTLE_MS: wait after release before sending AT commands
 *------------------------------------------------------------------------------*/
#define IOT_RESET_MS            (150u)
#define IOT_SETTLE_MS           (500u)

/*------------------------------------------------------------------------------
 * ^ command parser
 *   CMD_CHAR      – start-of-FRAM-command sentinel
 *   CMD_BUF_SIZE  – max command length including terminating 0x0D
 *------------------------------------------------------------------------------*/
#define CMD_CHAR                ('^')
#define CMD_TERMINATOR          (0x0Du)   /* Carriage Return                  */
#define CMD_BUF_SIZE            (32u)

/*------------------------------------------------------------------------------
 * Externs – IOT bridge ring buffers  (defined in serial.c)
 *
 * IOT_2_PC  – chars received from IOT (UCA0 RX), waiting to go to PC (UCA1 TX)
 * PC_2_IOT  – chars received from PC  (UCA1 RX), waiting to go to IOT (UCA0 TX)
 *
 * *_wr  – written by ISR
 * *_rd  / direct_*  – read by foreground or TX ISR
 *------------------------------------------------------------------------------*/
extern volatile unsigned char IOT_2_PC[SMALL_RING_SIZE];
extern volatile unsigned int  iot_rx_wr;
extern          unsigned int  iot_rx_rd;
extern          unsigned int  direct_iot;

extern volatile unsigned char PC_2_IOT[SMALL_RING_SIZE];
extern volatile unsigned int  usb_rx_wr;
extern          unsigned int  usb_rx_rd;
extern          unsigned int  direct_usb;

/*------------------------------------------------------------------------------
 * PC TX gate: set FALSE at init, set TRUE when first char received from PC.
 * FRAM never transmits to PC until this is TRUE.
 *------------------------------------------------------------------------------*/
extern volatile unsigned char pc_tx_enabled;

/*------------------------------------------------------------------------------
 * ^ command state (used by ISR and foreground)
 *   cmd_active   – TRUE while collecting a ^ command
 *   cmd_buf[]    – command characters (excludes the leading ^)
 *   cmd_len      – number of chars in cmd_buf
 *   cmd_ready    – TRUE when 0x0D received and command is complete
 *------------------------------------------------------------------------------*/
extern volatile unsigned char cmd_active;
extern volatile unsigned char cmd_buf[CMD_BUF_SIZE];
extern volatile unsigned int  cmd_len;
extern volatile unsigned char cmd_ready;

/*------------------------------------------------------------------------------
 * Legacy display buffer (kept for LCD line 0 in passthrough view)
 *------------------------------------------------------------------------------*/
extern volatile char          rx_display_buf[RX_DISPLAY_LEN + 1u];
extern volatile unsigned char rx_display_updated;

extern volatile unsigned char current_baud;
extern volatile unsigned char iot_baud;     /* tracks UCA0 baud separately    */

/*------------------------------------------------------------------------------
 * Function prototypes
 *------------------------------------------------------------------------------*/
void Init_Serial_UCA0(unsigned char baud);
void Init_Serial_UCA1(unsigned char baud);
void Set_Baud_UCA0(unsigned char baud);
void Set_Baud_UCA1(unsigned char baud);
void UCA1_Transmit_String(const char *str, unsigned int len);
void UCA0_Transmit_String(const char *str, unsigned int len);  /* to IOT      */
void IOT_Reset(void);                                           /* HW reset    */
void IOT_Send_Command(const char *cmd, unsigned int len);      /* cmd + CR/LF */

#endif /* SERIAL_H_ */
