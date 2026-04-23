/*------------------------------------------------------------------------------
 * File:        menus.h
 * Author:      [Student Name]
 * Target:      MSP430FR2355
 * Date:        2025
 *
 * Description: Header for the three scrolling menu system.
 *              Declares menu state constants, the active menu tracker,
 *              and all public menu functions used in main.c.
 *
 *              Menu system overview:
 *                MENU_MAIN      – top-level: Resistors / Shapes / Song
 *                MENU_RESISTORS – 10 resistor color codes (Table 1)
 *                MENU_SHAPES    – 10 shapes, big-middle 3-line display (Table 2)
 *                MENU_SONG      – Red and White lyrics, scrolling on big line
 *
 *              Navigation:
 *                Thumb wheel (ADC_Thumb) – scroll within current menu
 *                SW1 (Button 1)          – select / enter sub-menu
 *                SW2 (Button 2)          – back to main menu
 *------------------------------------------------------------------------------*/

#ifndef MENUS_H_
#define MENUS_H_

#include "macros.h"

/*------------------------------------------------------------------------------
 * Menu ID constants
 *------------------------------------------------------------------------------*/
#define MENU_SPLASH             (0u)   /* Startup splash screen                */
#define MENU_MAIN               (1u)   /* Top-level menu                       */
#define MENU_RESISTORS          (2u)   /* Resistor color code sub-menu         */
#define MENU_SHAPES             (3u)   /* Shapes sub-menu (big-mid display)    */
#define MENU_SONG               (4u)   /* Red and White song scroll            */

/*------------------------------------------------------------------------------
 * Main-menu item indices (3 items -> ADC 12-bit >> 11 = 0-1)
 * We want exactly 3 items; easiest: divide 1024 (post-shift 10-bit) by 3.
 * Range per item = 1024 / 3 = 341 counts (using scaled 10-bit ADC_Thumb).
 *
 * MAIN_ITEM_RESISTORS  0  (  0 – 340)
 * MAIN_ITEM_SHAPES     1  (341 – 681)
 * MAIN_ITEM_SONG       2  (682 – 1023)
 *------------------------------------------------------------------------------*/
#define MAIN_ITEM_RESISTORS     (0u)
#define MAIN_ITEM_SHAPES        (1u)
#define MAIN_ITEM_SONG          (2u)
#define MAIN_ITEM_COUNT         (3u)
#define MAIN_ITEM_RANGE         (341u)  /* 1024 / 3, last bucket absorbs slack */

/*------------------------------------------------------------------------------
 * Resistor menu: 10 items.
 * ADC_Thumb (0-1023) >> 7 = 0-7 only covers 8, so use division by 102:
 *   item = ADC_Thumb / 102  (items 0-9; item 9 catches 918-1023)
 *------------------------------------------------------------------------------*/
#define RESISTOR_ITEM_COUNT     (10u)
#define RESISTOR_ITEM_RANGE     (102u)  /* 1024 / 10, last bucket absorbs slack */

/*------------------------------------------------------------------------------
 * Shapes menu: 10 items (same ranges as resistors).
 *------------------------------------------------------------------------------*/
#define SHAPES_ITEM_COUNT       (10u)
#define SHAPES_ITEM_RANGE       (102u)

/*------------------------------------------------------------------------------
 * Song menu scrolling parameters.
 *   The song string has SONG_CHAR_COUNT characters.
 *   The thumb wheel cycles through SONG_SCROLL_ZONES "zones".  Each time the
 *   wheel crosses from zone N back up to zone 0 (a full CW sweep), the song
 *   advances by one character.  Moving CW has no effect.
 *   SONG_ZONE_SIZE: divide the 0-1023 ADC range into equal zones.
 *------------------------------------------------------------------------------*/
#define SONG_ZONE_COUNT         (8u)    /* 8 zones across thumb range           */
#define SONG_ZONE_SIZE          (128u)  /* 1024 / 8 = 128 counts per zone       */
#define SONG_DISPLAY_WIDTH      (10u)   /* Characters visible on one LCD line   */

/*------------------------------------------------------------------------------
 * Global: active menu.  Written by menu functions, read by main.c.
 *------------------------------------------------------------------------------*/
extern volatile unsigned char active_menu;

/*------------------------------------------------------------------------------
 * Function prototypes
 *------------------------------------------------------------------------------*/
void Menu_Init(void);
void Menu_Splash(void);
void Menu_Main(void);
void Menu_Resistors(void);
void Menu_Shapes(void);
void Menu_Song(void);
void Menu_Process(void);

#endif /* MENUS_H_ */
