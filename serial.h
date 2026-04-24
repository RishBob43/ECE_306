/*------------------------------------------------------------------------------
 * File:        serial.h
 * Target:      MSP430FR2355
 * Project 9 – IOT bridge approach matching Init_IOT / iot_rx_msg pattern
 *------------------------------------------------------------------------------*/

#ifndef SERIAL_H_
#define SERIAL_H_

#include "msp430.h"

/*------------------------------------------------------------------------------
 * Ring buffer
 *------------------------------------------------------------------------------*/
#define SERIAL_RING_SIZE        (64u)
#define SERIAL_BEGINNING        (0u)

/*------------------------------------------------------------------------------
 * Baud selectors
 *------------------------------------------------------------------------------*/
#define BAUD_115200             (0u)
#define BAUD_460800             (1u)
#define BAUD_9600               (2u)

/*------------------------------------------------------------------------------
 * Baud register values  (SMCLK = 8 MHz, UCOS16 = 1)
 *------------------------------------------------------------------------------*/
#define BR_115200_UCBRx         (4u)
#define BR_115200_MCTLW         (0x5551u)

#define BR_460800_UCBRx         (1u)
#define BR_460800_MCTLW         (0x4A11u)

#define BR_9600_UCBRx           (52u)
#define BR_9600_MCTLW           (0x4911u)

/*------------------------------------------------------------------------------
 * Display / timing
 *------------------------------------------------------------------------------*/
#define RX_DISPLAY_LEN          (10u)
#define SPLASH_TICKS            (25u)   /* 5 s  @ 200 ms per tick             */

#define IOT_MSG_SIZE            (64u)
#define IOT_MSG_COUNT           (2u)   /* double buffer */

extern volatile char          iot_rx_msg[IOT_MSG_COUNT][IOT_MSG_SIZE];
extern volatile unsigned char iot_msg_ready[IOT_MSG_COUNT];
extern volatile unsigned char iot_rx_write_idx;  /* ISR writes to this slot  */
extern volatile unsigned char iot_rx_read_idx;   /* foreground reads this     */
extern volatile unsigned int  iot_rx_count;

/*------------------------------------------------------------------------------
 * FRAM ^ command buffer size
 *------------------------------------------------------------------------------*/
#define FRAM_CMD_BUF_SIZE       (11u)

/*------------------------------------------------------------------------------
 * Externs – defined in serial.c
 *------------------------------------------------------------------------------*/
/* UCA0 ring */
extern volatile unsigned int  uca0_rx_wr;
extern volatile unsigned int  uca0_rx_rd;
extern volatile char          UCA0_Char_Rx[SERIAL_RING_SIZE];

/* UCA1 ring */
extern volatile unsigned int  uca1_rx_wr;
extern volatile unsigned int  uca1_rx_rd;
extern volatile char          UCA1_Char_Rx[SERIAL_RING_SIZE];

/* PC ready flag – set TRUE when first char received from PC                  */
extern volatile unsigned char pc_ready;

/* FRAM ^ command parser state */
extern volatile unsigned char fram_cmd_active;
extern volatile unsigned char fram_cmd_ready;
extern volatile unsigned int  fram_cmd_count;
extern volatile char          fram_cmd_buf[FRAM_CMD_BUF_SIZE];

/* LCD rolling display buffer */
extern volatile char          rx_display_buf[RX_DISPLAY_LEN + 1u];
extern volatile unsigned char rx_display_updated;

/* Current baud */
extern volatile unsigned char current_baud;

/*------------------------------------------------------------------------------
 * Function prototypes
 *------------------------------------------------------------------------------*/
void Init_Serial_UCA0(unsigned char baud);
void Init_Serial_UCA1(unsigned char baud);
void Set_Baud_UCA0(unsigned char baud);
void Set_Baud_UCA1(unsigned char baud);
void UCA1_Transmit_String(const char *str, unsigned int len);
void UCA0_Transmit_String(const char *str, unsigned int len);
void Init_IOT(void);

#endif /* SERIAL_H_ */
