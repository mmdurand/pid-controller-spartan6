/*****************************************************************************
*
* Copyright Roy Kravitz, Portland State University 2013, 2014, 2015
*
* Filename:          n3eif.h
* Version:           1.00.a
* Description:       n3eif Driver Header File (API)
* Date:              Sun 17-Mar-13 (by Roy Kravitz)
*
* NOTE:  This driver assumes that a Digilent PmodCLP (2 x 16 LCD) is plugged into
* two (2) of the Pmod headers on the Nexys3 board and that a Digilent PmodENC
* is plugged into a third (1) Pmod header on the Nexys3.  There is no errot checking
* done by the driver to verify this, though
*****************************************************************************/

#ifndef N3EIF_H
#define N3EIF_H

/***************************** Include Files *******************************/
#include "stdbool.h"
#include "n3eif_l.h"

/***************************** Constant Definitions ************************/

/**
 *  bit masks for buttons and switch register
 *  BTN_south is hardwired to system reset in EDK configurations
 */                  
#define	msk_SWITCH0		(0x01)
#define	msk_SWITCH1		(0x02)
#define	msk_SWITCH2		(0x04)
#define	msk_SWITCH3		(0x08)
#define	msk_BTN_WEST	(0x10)
#define	msk_BTN_EAST	(0x20)
#define	msk_BTN_NORTH	(0x40)
#define	msk_BTN_ROT		(0x80)

/***************************** Function Prototypes *************************/

// APIs for basic functionality

/**
 * N3EIF_init() - Initialize the Nexys 3 peripheral driver.  This function waits until
 * the n3eif self test is done then it sets the rotary encoder mode and clears the
 * rotary encoder count.  The LCD display is cleared on exit
 *
 * @param	BaseAddress is the base address in memory mapped I/O space for the peripheral
 *
 * @return	XST_SUCCESS
 *
 */
XStatus N3EIF_init(u32 BaseAddress);


/**
 * NX3_writeleds() - Write the Nexys 3 LEDs with the contents of LED_Data
 *
 * @param	LED_Data is the 8-bit value for the LEDs.  A 1 turns the LED on, 0 turns the LED off
 *
 * @return	XST_SUCCESS
 *
 * @note	Only the low-order 8-bits are used
 *
 */
XStatus NX3_writeleds(u32 LED_Data);


/**
 * NX3_readBtnSw() - Reads the buttons and switches from the Nexys 3 board and returns
 * them in BtnSw_Data
 *
 * @param	*BtnSw_Data is the address of an unsiged  32-bit register.  The low order
 *			8-bits contains the button/switch data.  See the #defines earlier in this
 *			file to see how the bits are mapped.
 *
 * @return	XST_SUCCESS
 *
 * @note	The button and switch value in in the low-order 8-bits
 *
 */
XStatus NX3_readBtnSw(u32 *BtnSw_Data);


/**
 * ROT_init() - Initializes the Rotary Encoder control.  "inc_dec_cnt" is the
 * increment/decrement count.  The count is truncated to 4 bits.  "no_neg"
 * TRUE says the rotary encoder count stops at 0.  "no_neg" FALSE says
 * the rotary encoder count can go negative.
 *
 * @param	inc_dec_cnt is the count for how much the rotary encorder increments
 *			or decrements each time the rotary encoder is turned
 * @param	no_neg permits or prevents the rotary count from going below 0.,  no_neg
 *			TRUE say do not allow negative counts.
 *
 * @return	XST_SUCCESS
 *
 */
XStatus ROT_init(int inc_dec_cnt, bool no_neg);

/**
 * ROT_clear() - Clears the Rotary Encoder count.  Does not affect
 * the rotary encoder set-up.  use ROT_init() to change
 * the rotary encoder ste-up
 *
 * @param	NONE
 *
 * @return	XST_SUCCESS
 *
 */
XStatus ROT_clear(void);

/**
 * ROT_readRotCnt() - Read the rotary encoder count into RotaryCnt
 *
 * @param	*RotaryCnt
 *
 * @return	XST_SUCCESS
 *
 */
XStatus ROT_readRotcnt(int* RotaryCnt);


/**								
 * LCD_docmd() - executes an LCD command
 *
 * Executes the LCD command in "lcdcmd" using the data
 * in "lcdata".  Controls the handshaking between the
 * user's command and the N3EIF Interface.
 *
 * @param  lcdcmd ia rhw LCD command to execute
 * @param  lcddata is the data for the command.  Not all commands have data
 *
 * @return XST_SUCCESS
 *
 * @notes only the lower 8-bits of lcdcmd and lcdata are used
*/
XStatus LCD_docmd(u32 lcdcmd, u32 lcddata);

/**								
 * LCD_setcursor() - set LCD cursor position to {line, col}
 *
 * Position the cursor ready for characters to be written.
 * The display is formed of 2 lines of 16 characters and each
 * position has a corresponding address as indicated below.
 *
 *                   Character position
 *           0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
 *
 * Line 1 - 80 81 82 83 84 85 86 87 88 89 8A 8B 8C 8D 8E 8F
 * Line 2 - C0 C1 C2 C3 C4 C5 C6 C7 C8 C9 CA CB CC CD CE CF
 *
 * @param line is the row on the display to set the cursor to.  There are two lines
 * @param col in the character position on the line.  There are 16 characters per line
 *
 * @return XST_SUCCESS
 *
 */
 XStatus LCD_setcursor(u32 row, u32 col);
 
/**
 * LCD_wrchar() - Write a character to the LCD display
 *
 * Writes a ASCII character "ch" to the LCD display
 * at the current cursor position
 *
 * @param ch is the character to write to the display
 *
 * @return XST_SUCCESS
 *
 */
 XStatus LCD_wrchar(char ch);
 
 
/**
 * LCD_shiftl() - Shifts the entire display left
 *
 * Shifts the display left one position without changing display RAM contents.
 * When the displayed data is shifted repeatedly, both lines move horizontally.
 * The second display line does not shift into the first display line.
 *
 * @param NONE
 *
 * @return XST_SUCCESS
 *
 */
 XStatus LCD_shiftl(void);
 
 /**
 * LCD_shiftr() - Shifts the entire display right
 *
 * Shifts the display right one position without changing display RAM contents.
 * When the displayed data is shifted repeatedly, both lines move horizontally.
 * The second display line does not shift into the first display line.
 *
 * @param NONE
 *
 * @return XST_SUCCESS
 *
 */
 XStatus LCD_shiftr(void);
 
 /**
 * LCD_setcgadr() - Set the character generator RAM Address
 *
 * Sets the CG RAM address to the value in "addr".  The function
 * also serves the purpose of telling the LCD controller that
 * the character data should be written to the character generator RAM instead
 * of the data RAM.  The character generator RAM contains 8 user defined custom
 * characters
 *
 * @param addr is the address in tha character generator ROM
 *
 * @return XST_SUCCESS
 *
 * @note:  only the low order 6-bits of the address are used
 */
 XStatus LCD_setcgadr(u32 addr);
 
 /**
 * LCD_setddadr() - Set the data RAM Address
 *
 * Sets the data RAM address to the value in "addr".  The function
 * also serves the purpose of telling the LCD controller that
 * the character data should be written to the display RAM instead
 * of the character generator RAM.  
 *
 * @param addr is the address in the diplay RAM
 *
 * @return XST_SUCCESS
 *
 * @note use LCD_setcursor() to set the position using {row, col} addressing
 * @note only the low order 7-bits of the address are used
 */
 XStatus LCD_setddadr(u32 addr);
 
/**
 * LCD_clrd() - Clear the display
 *
 * Writes blanks to the display and returns the cursor home.
 *
 * @param  NONE
 *
 * @return XST_SUCCESS
 *
 */
 XStatus LCD_clrd(void);
 
 
 // APIs for string related display functions
 
 /**
 * LCD_itoa() - converts an integer to ascii
 *
 * algorithm borrowed from ReactOS system libraries
 *
 * converts the integer "value" to ascii in base "radix".  *string is a pointer to the
 * string that the result is returned to. Returns the pointer to the string.
 * Assumes string[] is long enough to hold the result plus the terminating null
 *
 * @param 	value is the integer to convert
 * @param   *string is the address of buffer large enough to hold the converted number plus
 *          the terminating null
 * @param	radix is the base to use in conversion, 
 *
 * @return  pointer to the return string
*/
char* LCD_itoa(int value, char *string, int radix);
 
 
/**
 * LCD_wrstring() - Write a 0 terminated string to the LCD display
 *
 * Writes the null terminated string "s" to the LCD display
 * starting at the current cursor position
 *
 * @param *s is a pointer to the string to display
 *
 * @return XST_SUCCESS
 *
 */
 XStatus LCD_wrstring(char* s);
 
 
/**
 * LCD_puthex() - Write a 32-bit unsigned hex number to LCD display
 *       
 * Writes the 32-bit unsigned number "num" to the LCD display at the current
 * cursor position.
 *
 * @param num -is the number to display as a hex value
 *
 * @return  XST_SUCCESS
 */ 
 XStatus LCD_puthex(u32 num);
 
 
/**
 * LCD_putnum() - Write a 32-bit number in Radix "radix" to LCD display
 *
 * Writes the 32-bit number "num" to the LCD display at the current
 * cursor position. "radix" is the base to output the number in.
 *
 * @param num is the number to display
 * @param radix is the radix to display number in
 *
 * @return XST_SUCCESS
 */
XStatus LCD_putnum(int num, int radix);
 
#endif // N2EIF_H
