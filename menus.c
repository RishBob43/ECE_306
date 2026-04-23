/*------------------------------------------------------------------------------
 * File:        menus.c
 * Author:      [Student Name]
 * Date:        2025
 * Target:      MSP430FR2355
 * Compiler:    TI Code Composer Studio
 *
 * Description: Three-menu scrolling system driven by the ADC thumbwheel
 *              (ADC_Thumb, 0-1023 after >>2 scale) and two push buttons.
 *
 *              Menu navigation overview
 *              ─────────────────────────
 *              SPLASH      First Name / Last Name / "Homework 9" displayed in
 *                          4-line mode until SW1 or SW2 is pressed.
 *
 *              MAIN MENU   Three items: Resistors | Shapes | Song
 *                          Thumb wheel selects item.
 *                          SW1 enters the highlighted sub-menu.
 *                          SW2 has no effect (already at top level).
 *
 *              RESISTORS   10 resistor color codes (Table 1).
 *                          4-line display:
 *                            Line 0  "Color     "  (header)
 *                            Line 1  color name
 *                            Line 2  "Value     "  (header)
 *                            Line 3  digit character
 *                          Thumb wheel scrolls colors; menu stops at last item.
 *                          SW1 has no action.
 *                          SW2 returns to MAIN MENU.
 *
 *              SHAPES      10 shapes (Table 2).
 *                          Big-middle 3-line display (lcd_BIG_mid()):
 *                            Line 0  previous shape (or blank at top)
 *                            Line 1  CURRENT shape  (large centre line)
 *                            Line 2  next shape (or blank at bottom)
 *                          Thumb wheel scrolls shapes; menu stops at ends.
 *                          SW1 has no action.
 *                          SW2 returns to MAIN MENU and restores lcd_4line().
 *
 *              SONG        Red and White lyrics (Table 3) scrolled character
 *                          by character on the big centre line.
 *                          Big-middle 3-line display:
 *                            Line 0 / Line 2 alternate "Red and White" /
 *                                           "White and Red" banners.
 *                            Line 1  10-character sliding window into the song.
 *                          Characters advance only when the thumb wheel moves
 *                          counter-clockwise (decreasing ADC value crossing a
 *                          zone boundary).  Clockwise movement has no effect.
 *                          SW2 returns to MAIN MENU and restores lcd_4line().
 *
 * Globals modified:
 *   active_menu        – current menu ID (MENU_SPLASH … MENU_SONG)
 *   display_line[][]   – LCD text buffers
 *   display_changed    – set TRUE whenever display_line is updated
 *   sw1_pressed        – cleared after consumption
 *   sw2_pressed        – cleared after consumption
 *
 * Parameters passed in:  none (all state is in module-level statics or externs)
 * Return value:          none
 *
 * Notes:
 *   ADC_Thumb is the 10-bit scaled thumbwheel reading (0-1023) maintained
 *   by the ADC ISR in adc.c.  It is read directly; no critical-section guard
 *   is needed because it is a single-word volatile read on this architecture.
 *------------------------------------------------------------------------------*/

#include "msp430.h"
#include <string.h>
#include "functions.h"
#include "LCD.h"
#include "ports.h"
#include "macros.h"
#include "ADC.h"
#include "menus.h"

/*==============================================================================
 * Externs from globals.c / other translation units
 *==============================================================================*/
extern char                   display_line[4][11];
extern char                  *display[4];
extern volatile unsigned char display_changed;
extern volatile unsigned char update_display;
extern volatile unsigned char sw1_pressed;
extern volatile unsigned char sw2_pressed;

/*==============================================================================
 * Module-level global: tracks which menu is currently active.
 * Initialised to MENU_SPLASH by Menu_Init().
 * Extern'd via menus.h so main.c can gate button handling.
 *==============================================================================*/
volatile unsigned char active_menu = MENU_SPLASH;

/*==============================================================================
 * Static (file-scope) state variables
 *==============================================================================*/

/* ---- Resistor menu ---- */
static unsigned char  res_last_index = 0xFF;   /* Forces first draw             */

/* ---- Shapes menu ---- */
static unsigned char  shp_last_index = 0xFF;
static unsigned char  shp_big_active = FALSE;  /* TRUE while lcd_BIG_mid active */

/* ---- Song menu ---- */
static unsigned int   song_char_pos   = 0u;    /* Index of leftmost visible char */
static unsigned char  song_last_zone  = 0xFF;  /* Last ADC zone seen             */
static unsigned char  song_big_active = FALSE;
static unsigned char  song_alt_banner = FALSE; /* Alternates line 0/2 banner     */
static unsigned int   song_last_drawn = 0xFFFFu; /* Avoids redundant redraws     */

/* ---- Main menu ---- */
static unsigned char  main_last_item  = 0xFF;

/*==============================================================================
 * Constant data tables
 *==============================================================================*/

/* Resistor color names – padded to exactly 10 chars (null at [10]) */
static const char * const resistor_color[10] = {
    "Black     ",
    "Brown     ",
    "Red       ",
    "Orange    ",
    "Yellow    ",
    "Green     ",
    "Blue      ",
    "Violet    ",
    "Gray      ",
    "White     "
};

/* Resistor digit values as single character strings */
static const char resistor_digit[10] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
};

/* Shape names – padded to 10 chars */
static const char * const shape_name[10] = {
    "Circle    ",
    "Square    ",
    "Triangle  ",
    "Octagon   ",
    "Pentagon  ",
    "Hexagon   ",
    "Cube      ",
    "Oval      ",
    "Sphere    ",
    "Cylinder  "
};

/* Main menu item names */
static const char * const main_item_name[3] = {
    "Resistors ",
    "Shapes    ",
    "Song      "
};

/*
 * Red and White song lyrics (Table 3).
 * Stored as a single flat string; song_char_pos indexes into it.
 * The display window is 10 characters wide and scrolls right-to-left.
 *
 * Full text (spaces preserved exactly as given):
 *   "We're the Red and White from State "
 *   "And we know we are the best. "
 *   "A hand behind our back, "
 *   "We can take on all the rest."
 *   "Come over the hill, Carolina. "
 *   "Devils and Deacs stand in line. "
 *   "The Red and White from N.C. State. "
 *   "Go State!"
 *
 * Leading and trailing spaces provide the scroll-in / scroll-out effect.
 */
static const char song_lyrics[] =
    "          "                              /* 10-char lead-in blank           */
    "We're the Red and White from State "
    "And we know we are the best. "
    "A hand behind our back, "
    "We can take on all the rest. "
    "Come over the hill, Carolina. "
    "Devils and Deacs stand in line. "
    "The Red and White from N.C. State. "
    "Go State!"
    "          ";                             /* 10-char trail-out blank          */

/* Total number of displayable positions (length of lyrics minus display width) */
#define SONG_TOTAL_CHARS  (sizeof(song_lyrics) - 1u)   /* exclude null terminator */

/*==============================================================================
 * Internal helpers
 *==============================================================================*/

/*------------------------------------------------------------------------------
 * set_display_line
 * Author: [Student Name]
 * Date:   2025
 *
 * Description: Copies up to 10 characters from src into display_line[line],
 *              null-terminates at position 10, and sets display_changed.
 *
 * Parameters:
 *   line  – display line index (0-3)
 *   src   – null-terminated source string (at most 10 printable chars used)
 *
 * Returns: nothing
 * Globals modified: display_line[line], display_changed
 *------------------------------------------------------------------------------*/
static void set_display_line(int line, const char *src){
    strncpy(display_line[line], src, DISPLAY_LINE_LENGTH - 1u);
    display_line[line][DISPLAY_LINE_LENGTH - 1u] = RESET_STATE;
    display_changed = TRUE;
}

/*------------------------------------------------------------------------------
 * clear_display_line
 * Author: [Student Name]
 * Date:   2025
 *
 * Description: Fills display_line[line] with spaces and sets display_changed.
 *
 * Parameters:  line – index 0-3
 * Returns:     nothing
 * Globals modified: display_line[line], display_changed
 *------------------------------------------------------------------------------*/
static void clear_display_line(int line){
    set_display_line(line, "          ");
}

/*------------------------------------------------------------------------------
 * adc_to_menu_index
 * Author: [Student Name]
 * Date:   2025
 *
 * Description: Maps a 10-bit ADC value (0-1023) to a zero-based menu index
 *              by integer division.  The result is clamped to [0, count-1]
 *              to handle the edge case where adc_val == 1023 and range * count
 *              slightly exceeds 1023.
 *
 * Parameters:
 *   adc_val  – ADC_Thumb value (0-1023)
 *   range    – counts per item (e.g. 102 for 10 items)
 *   count    – total number of items
 *
 * Returns: unsigned char index in [0, count-1]
 * Globals modified: none
 *------------------------------------------------------------------------------*/
static unsigned char adc_to_menu_index(unsigned int adc_val,
                                       unsigned int range,
                                       unsigned char count)
{
    unsigned char idx = (unsigned char)(adc_val / range);
    if(idx >= count){
        idx = (unsigned char)(count - 1u);
    }
    return idx;
}

/*==============================================================================
 * Menu_Init
 * Author: [Student Name]
 * Date:   2025
 *
 * Description: Resets all menu state variables to their initial values and
 *              sets active_menu to MENU_SPLASH.  Call once after Init_LCD()
 *              and before entering the main loop.
 *
 * Parameters passed in:  none
 * Returns:                nothing
 * Globals modified: active_menu, res_last_index, shp_last_index,
 *                   shp_big_active, song_char_pos, song_last_zone,
 *                   song_big_active, song_alt_banner, song_last_drawn,
 *                   main_last_item
 *==============================================================================*/
void Menu_Init(void){
    active_menu      = MENU_SPLASH;
    res_last_index   = 0xFF;
    shp_last_index   = 0xFF;
    shp_big_active   = FALSE;
    song_char_pos    = 0u;
    song_last_zone   = 0xFF;
    song_big_active  = FALSE;
    song_alt_banner  = FALSE;
    song_last_drawn  = 0xFFFFu;
    main_last_item   = 0xFF;
}

/*==============================================================================
 * Menu_Splash
 * Author: [Student Name]
 * Date:   2025
 *
 * Description: Displays the startup splash screen in 4-line mode:
 *                Line 0 – student first name (10 chars, space-padded)
 *                Line 1 – (blank – space for personalisation)
 *                Line 2 – student last name  (10 chars, space-padded)
 *                Line 3 – "Homework 9"
 *              Transitions to MENU_MAIN when SW1 or SW2 is pressed.
 *
 *              NOTE: Replace the placeholder strings with your actual name.
 *
 * Parameters passed in:  none
 * Returns:                nothing
 * Globals modified: active_menu, display_line, display_changed,
 *                   sw1_pressed, sw2_pressed
 *==============================================================================*/
void Menu_Splash(void){
    static unsigned char splash_drawn = FALSE;

    /* Draw splash once */
    if(!splash_drawn){
        lcd_4line();                           /* Ensure 4-line mode             */
        set_display_line(0, "FirstName ");    /* TODO: replace with your name   */
        set_display_line(1, "          ");
        set_display_line(2, "LastName  ");    /* TODO: replace with your name   */
        set_display_line(3, "Homework 9");
        Display_Update(0, 0, 0, 0);
        splash_drawn = TRUE;
    }

    /* Wait for either button */
    if(sw1_pressed || sw2_pressed){
        sw1_pressed  = FALSE;
        sw2_pressed  = FALSE;
        splash_drawn = FALSE;   /* Reset for next call if ever re-entered       */
        active_menu  = MENU_MAIN;
        main_last_item = 0xFF;  /* Force immediate redraw of main menu          */
    }
}

/*==============================================================================
 * Menu_Main
 * Author: [Student Name]
 * Date:   2025
 *
 * Description: Top-level 3-item scrolling menu.  The thumbwheel selects one
 *              of three items (Resistors / Shapes / Song).  The LCD shows:
 *                Line 0 – "  MENU    "  (static header)
 *                Line 1 – currently highlighted item name
 *                Line 2 – "  ------  "  (static separator)
 *                Line 3 – "SW1=Select"  (static hint)
 *
 *              SW1 pressed  -> enter the selected sub-menu.
 *              SW2 pressed  -> no action (already at top level).
 *
 *              The display is only rewritten when the selected item changes,
 *              avoiding unnecessary SPI bus traffic.
 *
 * Parameters passed in:  none
 * Returns:                nothing
 * Globals modified: active_menu, display_line, display_changed,
 *                   sw1_pressed, sw2_pressed, main_last_item
 *==============================================================================*/
void Menu_Main(void){
    unsigned char item;

    /* Map thumb wheel to one of 3 items */
    item = adc_to_menu_index(ADC_Thumb, MAIN_ITEM_RANGE, MAIN_ITEM_COUNT);

    /* Redraw only when item changes */
    if(item != main_last_item){
        main_last_item = item;
        lcd_4line();
        set_display_line(0, "  MENU    ");
        set_display_line(1, main_item_name[item]);
        set_display_line(2, "  ------  ");
        set_display_line(3, "SW1=Select");
        Display_Update(0, 0, 0, 0);
    }

    /* SW1: enter sub-menu */
    if(sw1_pressed){
        sw1_pressed = FALSE;
        switch(item){
            case MAIN_ITEM_RESISTORS:
                res_last_index = 0xFF;          /* Force immediate draw          */
                active_menu = MENU_RESISTORS;
                break;
            case MAIN_ITEM_SHAPES:
                shp_last_index = 0xFF;
                active_menu = MENU_SHAPES;
                break;
            case MAIN_ITEM_SONG:
                song_char_pos   = 0u;
                song_last_zone  = 0xFF;
                song_last_drawn = 0xFFFFu;
                song_alt_banner = FALSE;
                active_menu = MENU_SONG;
                break;
            default:
                break;
        }
    }

    /* SW2: no action at top level (consume the press to avoid re-entry) */
    if(sw2_pressed){
        sw2_pressed = FALSE;
    }
}

/*==============================================================================
 * Menu_Resistors
 * Author: [Student Name]
 * Date:   2025
 *
 * Description: Displays resistor color code information for the color
 *              selected by the thumbwheel.  Uses standard 4-line mode.
 *
 *              LCD layout:
 *                Line 0 – "Color     "  (static header)
 *                Line 1 – color name    (e.g. "Red       ")
 *                Line 2 – "Value     "  (static header)
 *                Line 3 – resistor digit value as a single character
 *
 *              Scrolling: the 10-bit ADC_Thumb value is divided into 10
 *              equal ranges.  The menu stops at item 0 (bottom stop) and
 *              item 9 (top stop) – it does not wrap.
 *
 *              SW1  – no action in this sub-menu.
 *              SW2  – return to MENU_MAIN.
 *
 * Parameters passed in:  none
 * Returns:                nothing
 * Globals modified: active_menu, display_line, display_changed,
 *                   sw1_pressed, sw2_pressed, res_last_index
 *==============================================================================*/
void Menu_Resistors(void){
    unsigned char idx;
    char val_str[11];

    /* Ensure 4-line mode (restores if coming from Shapes/Song) */
    lcd_4line();

    /* Map thumb to item */
    idx = adc_to_menu_index(ADC_Thumb, RESISTOR_ITEM_RANGE, RESISTOR_ITEM_COUNT);

    /* Only redraw when index changes */
    if(idx != res_last_index){
        res_last_index = idx;

        /* Build value string: single digit followed by 9 spaces */
        strncpy(val_str, "          ", 10u);  /* fill with spaces first         */
        val_str[0]  = resistor_digit[idx];    /* place digit at position 0      */
        val_str[10] = RESET_STATE;

        set_display_line(0, "Color     ");
        set_display_line(1, resistor_color[idx]);
        set_display_line(2, "Value     ");
        set_display_line(3, val_str);
        Display_Update(0, 0, 0, 0);
    }

    /* SW1: no action */
    if(sw1_pressed){
        sw1_pressed = FALSE;
    }

    /* SW2: back to main menu */
    if(sw2_pressed){
        sw2_pressed    = FALSE;
        main_last_item = 0xFF;  /* Force main menu redraw                       */
        active_menu    = MENU_MAIN;
    }
}

/*==============================================================================
 * Menu_Shapes
 * Author: [Student Name]
 * Date:   2025
 *
 * Description: Displays a shape name in the large middle line using the
 *              3-line big-middle LCD mode (lcd_BIG_mid()).
 *
 *              LCD layout (big-mid mode):
 *                Line 0 – previous shape name, or "          " at top of list
 *                Line 1 – CURRENT shape (displayed in large font)
 *                Line 2 – next shape name, or "          " at bottom of list
 *
 *              Scrolling: the 10-bit ADC_Thumb is divided into 10 equal
 *              ranges.  The menu stops at item 0 and item 9.
 *
 *              SW1  – no action in this sub-menu.
 *              SW2  – return to MENU_MAIN; restores 4-line mode.
 *
 * Parameters passed in:  none
 * Returns:                nothing
 * Globals modified: active_menu, display_line, display_changed,
 *                   sw1_pressed, sw2_pressed, shp_last_index, shp_big_active
 *==============================================================================*/
void Menu_Shapes(void){
    unsigned char idx;

    /* Switch to big-middle 3-line mode on first entry */
    if(!shp_big_active){
        lcd_BIG_mid();
        shp_big_active = TRUE;
        shp_last_index = 0xFF;  /* Force redraw                                 */
    }

    idx = adc_to_menu_index(ADC_Thumb, SHAPES_ITEM_RANGE, SHAPES_ITEM_COUNT);

    if(idx != shp_last_index){
        shp_last_index = idx;

        /* Line 0: previous shape or blank */
        if(idx == 0u){
            clear_display_line(0);
        } else {
            set_display_line(0, shape_name[idx - 1u]);
        }

        /* Line 1 (big centre): current shape */
        set_display_line(1, shape_name[idx]);

        /* Line 2: next shape or blank */
        if(idx >= (SHAPES_ITEM_COUNT - 1u)){
            clear_display_line(2);
        } else {
            set_display_line(2, shape_name[idx + 1u]);
        }

        Display_Update(0, 0, 0, 0);
    }

    /* SW1: no action */
    if(sw1_pressed){
        sw1_pressed = FALSE;
    }

    /* SW2: restore 4-line mode and return to main menu */
    if(sw2_pressed){
        sw2_pressed    = FALSE;
        shp_big_active = FALSE;
        lcd_4line();
        main_last_item = 0xFF;
        active_menu    = MENU_MAIN;
    }
}

/*==============================================================================
 * Menu_Song
 * Author: [Student Name]
 * Date:   2025
 *
 * Description: Scrolls the Red and White song lyrics on the large centre line
 *              of the big-middle 3-line display.
 *
 *              LCD layout (big-mid mode):
 *                Line 0 – alternates between "Red&White " and "White&Red "
 *                Line 1 – 10-character window into the lyrics (large font)
 *                Line 2 – alternates between "White&Red " and "Red&White "
 *                         (opposite to Line 0 so the two banners swap each
 *                          time a new character is shown)
 *
 *              Scrolling mechanism:
 *                The thumb wheel range (0-1023) is divided into SONG_ZONE_COUNT
 *                (8) equal zones of SONG_ZONE_SIZE (128) counts each.
 *
 *                Zone transitions from a HIGHER zone to a LOWER zone
 *                (counter-clockwise movement) advance song_char_pos by one.
 *                Zone transitions from lower to higher (clockwise) have no
 *                effect.
 *
 *                This means one full counter-clockwise sweep of the wheel
 *                (zone 7 -> 0) advances the song by one character.  The user
 *                sweeps CCW repeatedly to read through the lyrics.
 *
 *                song_char_pos is clamped so that the visible 10-char window
 *                never runs past the end of song_lyrics.
 *
 *              SW1  – no action.
 *              SW2  – return to MENU_MAIN; restores 4-line mode.
 *
 * Parameters passed in:  none
 * Returns:                nothing
 * Globals modified: active_menu, display_line, display_changed,
 *                   sw1_pressed, sw2_pressed, song_big_active,
 *                   song_char_pos, song_last_zone, song_alt_banner,
 *                   song_last_drawn
 *==============================================================================*/
void Menu_Song(void){
    unsigned char  zone;
    unsigned int   max_pos;
    char           window[11];
    unsigned int   i;
    unsigned int   src_idx;

    /* Switch to big-middle 3-line mode on first entry */
    if(!song_big_active){
        lcd_BIG_mid();
        song_big_active = TRUE;
        song_last_zone  = 0xFF;
        song_last_drawn = 0xFF;
    }

    /* Compute current zone (0 = low ADC = far CCW, 7 = high ADC = far CW) */
    zone = (unsigned char)(ADC_Thumb / SONG_ZONE_SIZE);
    if(zone >= SONG_ZONE_COUNT){
        zone = (unsigned char)(SONG_ZONE_COUNT - 1u);
    }

    /* Advance song only on CCW crossing (zone decreases) */
    if(song_last_zone != 0xFF){
        if(zone < song_last_zone){
            /* CCW movement detected – advance one character */
            max_pos = (unsigned int)(SONG_TOTAL_CHARS > SONG_DISPLAY_WIDTH
                      ? SONG_TOTAL_CHARS - SONG_DISPLAY_WIDTH : 0u);
            if(song_char_pos < max_pos){
                song_char_pos++;
                song_alt_banner = (unsigned char)(!song_alt_banner);
            }
        }
        /* CW movement: intentionally ignored per spec */
    }
    song_last_zone = zone;

    /* Redraw only when position has changed */
    if(song_char_pos != song_last_drawn){
        song_last_drawn = song_char_pos;   /* Record position to detect future changes */

        /* Build 10-char window into lyrics */
        for(i = 0u; i < SONG_DISPLAY_WIDTH; i++){
            src_idx = song_char_pos + i;
            if(src_idx < SONG_TOTAL_CHARS){
                window[i] = song_lyrics[src_idx];
            } else {
                window[i] = ' ';
            }
        }
        window[SONG_DISPLAY_WIDTH] = RESET_STATE;

        /* Alternating banners on Line 0 and Line 2 */
        if(song_alt_banner){
            set_display_line(0, "White&Red ");
            set_display_line(2, "Red&White ");
        } else {
            set_display_line(0, "Red&White ");
            set_display_line(2, "White&Red ");
        }

        set_display_line(1, window);
        Display_Update(0, 0, 0, 0);
    }

    /* SW1: no action */
    if(sw1_pressed){
        sw1_pressed = FALSE;
    }

    /* SW2: restore 4-line mode and return to main menu */
    if(sw2_pressed){
        sw2_pressed     = FALSE;
        song_big_active = FALSE;
        lcd_4line();
        main_last_item  = 0xFF;
        active_menu     = MENU_MAIN;
    }
}

/*==============================================================================
 * Menu_Process
 * Author: [Student Name]
 * Date:   2025
 *
 * Description: Dispatcher – call this once per pass of the main while loop.
 *              Routes execution to the currently active menu function based
 *              on the value of active_menu.
 *
 *              This is the only function that needs to be called from main.c;
 *              all menu state transitions are handled internally.
 *
 * Parameters passed in:  none
 * Returns:                nothing
 * Globals modified: active_menu (indirectly via sub-functions)
 *==============================================================================*/
void Menu_Process(void){
    switch(active_menu){
        case MENU_SPLASH:
            Menu_Splash();
            break;
        case MENU_MAIN:
            Menu_Main();
            break;
        case MENU_RESISTORS:
            Menu_Resistors();
            break;
        case MENU_SHAPES:
            Menu_Shapes();
            break;
        case MENU_SONG:
            Menu_Song();
            break;
        default:
            active_menu = MENU_SPLASH;
            break;
    }
}
