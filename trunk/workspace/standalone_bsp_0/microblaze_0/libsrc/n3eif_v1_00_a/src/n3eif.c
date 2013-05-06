/*****************************************************************************
*
* Copyright Roy Kravitz, Portland State University 2013, 2014, 2015
*
* Filename:          n3eif.c
* Version:           1.00.a
* Description:       n3eif Driver Header File (API)
* Date:              Sun 17-Mar-13 (by Roy Kravitz)
*
* NOTE:  This driver assumes that a Digilent PmodCLP (2 x 16 LCD) is plugged into
* two (2) of the Pmod headers on the Nexys3 board and that a Digilent PmodENC
* is plugged into a third (1) Pmod header on the Nexys3.  There is no errot checking
* done by the driver to verify this, though
*****************************************************************************/


/***************************** Include Files *******************************/

#include "n3eif.h"

/************************** Constant Declarations ***************************/

/**
 *  bit masks for rotary control and status register
 */

const u32	msk_ROTCTL_INTRVL	= 0x0000000F;	//  Rotary encoder inc/dec interval bits[3:0]
const u32	msk_ROTCTL_NONEG	= 0x00000010;	//  Force rotary count positive or 0
const u32	msk_ROTCTL_SP0		= 0x00000020;	//  Rotary control spare bit

const u32	msk_ROTCTL_LDSTATE	= 0x00000040;	//	Load new Rotary Control state
const u32	msk_ROTCTL_CLRCNT	= 0x00000080;	//  Clear rotary encoder count

const u32	msk_ROTSTS_BUSY		= 0x80000000;	//	Busy bit - 1=rotary state is being updated

const u32	msk_ROTSTS_RDY		= 0x00000000;	//	Busy bit - 0=rotary state is ready
const u32	msk_ROTSTS_STST		= 0x00000040;	//	Self-test - 1 = not ready for use

/**									
 *  bit masks for LCD command and status register
 */
 
const u32	msk_LCDCMD_DO		= 0x00000080;	//	Do new command.  program looks for rising edge
												//  on bit[7] to kick off new command
const u32	msk_LCDCMD_CMD		= 0x0000001F;	//	Bits[4:0] are LCD command request

const u32	msk_LCDSTS_BUSY		= 0x00000008;	//	Busy bit - 1= LCD command in progress
const u32	msk_LCDSTS_RDY		= 0x00000000;	//	Busy bit - 0= LCD is ready for a command

/**
 * LCD Controller command codes
 */
const u8	LCDCC_NOOP			= 0x00;	//	LCD command code - No operation
const u8	LCDCC_SETCURSOR		= 0x01;	//	LCD command code - Set cursor position to {line,row}
const u8	LCDCC_WRCHAR		= 0x02;	//	LCD command code - Write character to cursor position
const u8	LCDCC_RDCHAR		= 0x03;	//	LCD command code - Read character at cursor position

const u8	LCDCC_CLRD			= 0x04;	//	LCD command code - Clear Display
const u8	LCDCC_HOME			= 0x05;	//	LCD command code - Return Cursor Home
const u8	LCDCC_SETCGADR		= 0x06;	//	LCD command code - Set CG RAM Address
const u8	LCDCC_SETDDADR		= 0x07;	//	LCD command code - Set DD RAM Address

const u8	LCDCC_SETMODE		= 0x08;	//	LCD command code - Entry Mode Set
const u8	LCDCC_DSPLYONOFF	= 0x09;	//	LCD command code - Display On/Off
const u8	LCDCC_SHIFTL		= 0x0A;	//	LCD command code - Shift entire display left
const u8	LCDCC_SHIFTR		= 0x0B;	//	LCD command code - Shift entire display right
const u8	LCDCC_MOVECL		= 0x0C;	//	LCD command code - Move cursor left one character
const u8	LCDCC_MOVECR		= 0x0D;	//	LCD command code - MOve cursor right one character

/**************************** Global Variables *****************************/
static u32	BaseAddr;					// Base Address for n3eif registers
static bool	IsReady = false;			// n3eif peripheral has been successfully initialized

/************************** Function Definitions ***************************/
// N3EIF (Nexys 3 Extended Interface) Low Level Functions

/*****************************************************************************************/
/* N3EIF_usleep() - delay "usec" microseconds.  This function should be in libc but it   */
/* seems to be missing.  This emulation implements a delay loop with approximate timing; */
/* not perfect but it gets the job done. 												 */
/*                                                                                       */
/* This emulation assumes that the microblaze is running @ 50MHz and takes 1 to execute	 */
/* a NOP - this is probably OK when the program is in BRAM since it runs at the CPU      */
/* speed but may not be correct if the memory is external to the FPGA                    */
/*****************************************************************************************/

const u32	DELAY_1USEC_CONSTANT	= 50;	// constant for 1 microsecond delay

void N3EIF_usleep(u32 usec)
{
	volatile u32 count;
	
	for (count = 0; count < ((usec * DELAY_1USEC_CONSTANT) + 1); count++)
	{
		asm("nop");
	}
	return;
} 


/*****************************************************************************************/
/* N3EIF_init() - Initialize the Nexus 3 Expanded Interface This function waits until    */
/* the  hardware self test is done, runs the driver self-test (only the first time the   */
/* peripheral is intitialized) and if the self-test passes it ets the rotary encoder     */
/* and mode and clears the rotary encoder count.  Then it clears the LCD and returns     */
/*****************************************************************************************/

XStatus N3EIF_init(u32 BaseAddress)
{
	XStatus sts;
	
	// give interface 20ms to start up
	N3EIF_usleep(20000);
	
	// Save the Base Address so we know where to point the driver
	// Uses the global variable BaseAddr to store it.
	BaseAddr = BaseAddress;
	
	// Run the driver self-test.  Return on failure if it doesn't pass
	// We only do this the first time the n3eif peripheral is initialized
	//
	// Note:  The self-test does not read/write the Rotary Status registers
	// since it holds whether the hardware self-test has completed.
	if (!IsReady)
	{
		sts = N3EIF_SelfTest(BaseAddr);
		if (sts != XST_SUCCESS)
		{
			return XST_FAILURE;
		}
	}
	IsReady = true;
	
	// wait until n3eif hardware self-test is complete
	// this is indicated by Rotary Status register bit 6 == 0
	sts = N3EIF_mReadROTSTS(BaseAddr);
	while ((sts & msk_ROTSTS_STST) != 0)
	{  
		N3EIF_usleep(1000);  // wait 1ms and then try again
		sts = N3EIF_mReadROTSTS(BaseAddr);
	}
	
	// initialize the rotary encoder to incr/decr by 1, OK to go negative
	ROT_init(1, false);
	ROT_clear();
	
	// and clear the LCD display
	LCD_clrd();
	N3EIF_usleep(10);
		
	return XST_SUCCESS;	
}


// API Nexys 3 board peripherals

/*****************************************************************************************/
/* NX3_writeleds() - Write the LEDs with the contents of LED_Data                        */
/*****************************************************************************************/
XStatus NX3_writeleds(u32 LED_Data)
{
	u32	leds;
	
	leds = LED_Data & 0x000000FF;	// there are only 8-leds in bits[24:31] (big endian)
	N3EIF_mWriteLEDS_DATA(BaseAddr, leds);
	N3EIF_usleep(10);
	
	return XST_SUCCESS;	
}


/*****************************************************************************************/
/* NX3_readBntSw() - Read the buttons and switches into BtnSw_Data                       */
/*****************************************************************************************/
XStatus NX3_readBtnSw(u32 *BtnSw_Data)
{
	u32 btnsw;
	
	btnsw = N3EIF_mReadBTNSW_IN(BaseAddr);
	*BtnSw_Data = btnsw & 0x000000FF;
	N3EIF_usleep(10);
	
	return XST_SUCCESS;
}

// API for PmodENC (Rotary Encoder) peripheral

/*****************************************************************************************/
/* ROT_init() - Initializes the Rotary Encoder control.  "inc_dec_cnt" is the            */
/* increment/decrement count.  The count is truncated to 4 bits.  "no_neg"               */
/* TRUE says the rotary encoder count stops at 0.  "no_neg" FALSE says                   */
/* the rotary encoder count can go negative.                                             */
/*****************************************************************************************/
XStatus ROT_init(int inc_dec_cnt, bool no_neg)
{
	u32 sts, enc_state;

	// wait until rotary encoder control is ready - e.g. BUSY bit is 0	
	sts = N3EIF_mReadROTSTS(BaseAddr);
	while ((sts & msk_ROTSTS_BUSY) != 0)
	{  
		N3EIF_usleep(10);
		sts = N3EIF_mReadROTSTS(BaseAddr);
	}
	
	// build the new rotary encoder control state
	enc_state = (inc_dec_cnt & msk_ROTCTL_INTRVL);
	if (no_neg)
		enc_state |= msk_ROTCTL_NONEG;
	
	// kick off the command by writing 1 to	"Load State" bit
	enc_state = (enc_state | msk_ROTCTL_LDSTATE);
	N3EIF_mWriteROTCTL(BaseAddr, enc_state);
	
	// wait until command is complete - e.g. BUSY bit is 0 
	do  
	{
		N3EIF_usleep(10);
		sts = N3EIF_mReadROTSTS(BaseAddr);
	} while ((sts & msk_ROTSTS_BUSY) != 0);
	
	// end the command by toggling (writing 0) to "Load State" bit
	// and waiting a bit (5us) to make sure rotary encoder controller "sees" the falling edge
	N3EIF_mWriteROTCTL(BaseAddr, (enc_state ^ msk_ROTCTL_LDSTATE));
	N3EIF_usleep(10);
	
	return XST_SUCCESS;
}


/*****************************************************************************************/
/* ROT_clear() - Clears the Rotary Encoder count.  Does not affect
/* the rotary encoder set-up.  use ROT_init() to change
/* the rotary encoder set-up
/*****************************************************************************************/
XStatus ROT_clear(void)
{
	u32 sts;

	// wait until rotary encoder control is ready - e.g. BUSY bit is 0	
	sts = N3EIF_mReadROTSTS(BaseAddr);
	while ((sts & msk_ROTSTS_BUSY) != 0)
	{  
		N3EIF_usleep(10);
		sts = N3EIF_mReadROTSTS(BaseAddr);
	}
	
	
	// kick off the command by writing 1 to	"Clear Count" bit
	N3EIF_mWriteROTCTL(BaseAddr, msk_ROTCTL_CLRCNT);
	
	// wait until command is complete - e.g. BUSY bit is 0 
	do  
	{
		N3EIF_usleep(10);
		sts = N3EIF_mReadROTSTS(BaseAddr);
	} while ((sts & msk_ROTSTS_BUSY) != 0);
	
	// end the command by toggling (writing 0) to "Clear Count" bit
	// and waiting a bit (5us) to make sure rotary encoder controller "sees" the falling edge
	N3EIF_mWriteROTCTL(BaseAddr, 0x00000000);
	N3EIF_usleep(10);
	
	return XST_SUCCESS;
}

/*****************************************************************************************/
/* ROT_readRotCnt() - Read the rotary encoder count into RotaryCnt                       */
/*****************************************************************************************/

XStatus ROT_readRotcnt(int* RotaryCnt)
{
	u32 lo, hi;			// low and high count bytes
	int cnt;			// 32-bit count, lower 16-bits are valid

	// get the low and hi counts and combine them to form an integer
	lo = N3EIF_mReadROTCNTL(BaseAddr) & 0x000000FF;	
	hi = N3EIF_mReadROTCNTH(BaseAddr) & 0x000000FF;
	cnt = (hi * 256) + lo;
	*RotaryCnt = cnt;
	N3EIF_usleep(10);
	
	return XST_SUCCESS;
}

// API for PmodCLP (2 x 16 LCD display) peripheral

/*****************************************************************************************/								
/* LCD_docmd() - executes an LCD command                                                 */
/*                                                                                       */
/* Executes the LCD command in "lcdcmd" using the data in "lcdata".  Controls the        */ 
/* handshaking between the user's command and the S3E Interface.                         */
/*****************************************************************************************/

XStatus LCD_docmd(u32 lcdcmd, u32 lcddata)
{
	u32 sts, lcd_cmd;

	// wait until LCD controller is ready - e.g. BUSY bit is 0	
	sts = N3EIF_mReadLCDSTS(BaseAddr);
	while ((sts & msk_LCDSTS_BUSY) != 0)
	{  
		N3EIF_usleep(10);
		sts = N3EIF_mReadLCDSTS(BaseAddr);
	}
	
	// write the LCD data to the LCD controller
	N3EIF_mWriteLCDDATA(BaseAddr, lcddata);

	// write the LCD command into bits[4:0] of the LCD Command register
	lcd_cmd = lcdcmd & msk_LCDCMD_CMD;
	N3EIF_mWriteLCDCMD(BaseAddr, lcd_cmd);
		
	// kick off the command by writing 1 to	"Do LCD command" bit
	lcd_cmd |= msk_LCDCMD_DO;
	N3EIF_mWriteLCDCMD(BaseAddr, lcd_cmd);
	
	// wait until command is complete - e.g. BUSY bit is 0 
	do  
	{
		N3EIF_usleep(10);
		sts = N3EIF_mReadLCDSTS(BaseAddr);
	} while ((sts & msk_LCDSTS_BUSY) != 0);
	
	// end the command by toggling (writing 0) to "Do LCD command" bit
	// and waiting at least 1.53ms to make sure LCD display controller "sees" the falling edge
	// and that the LCD has finished the operations.  1.53ms is the worst case command timing
	lcd_cmd ^= msk_LCDCMD_DO;
	N3EIF_mWriteLCDCMD(BaseAddr, lcd_cmd);
	N3EIF_usleep(500);
	
	return XST_SUCCESS;
}

/*****************************************************************************************/								
/* LCD_setcursor() - set LCD cursor position to {line, col}                              */
/*                                                                                       */
/* Position the cursor ready for characters to be written.                               */
/* The display is formed of 2 lines of 16 characters.                                    */
/*****************************************************************************************/
XStatus LCD_setcursor(u32 row, u32 col)
{
	 u32 pos;
	 XStatus sts;
	 
	 pos = ((row & 0x0000000F) << 4) | (col & 0x0000000F);
	 sts = LCD_docmd(LCDCC_SETCURSOR, pos);
	 
	 return sts;
}

	   
/*****************************************************************************************/	
/* LCD_wrchar() - Write a character to the LCD display                                   */
/*                                                                                       */
/* Writes a ASCII character "ch" to the LCD display                                      */
/* at the current cursor position                                                        */
/*****************************************************************************************/	
XStatus LCD_wrchar(char ch)
{
	 XStatus sts;
	 
	 sts = LCD_docmd(LCDCC_WRCHAR, ch);
	 
	 return sts;
}

  
/*****************************************************************************************/
/* LCD_shiftl() - Shifts the entire display left                                         */
/*                                                                                       */
/* Shifts the display left one position without changing display RAM contents.           */
/* When the displayed data is shifted repeatedly, both lines move horizontally.          */
/* The second display line does not shift into the first display line.                   */
/*****************************************************************************************/
XStatus LCD_shiftl(void)
{
	XStatus sts;
	
	sts = LCD_docmd(LCDCC_SHIFTL, 0);
	
	return sts;
}

 
 /****************************************************************************************/
/* LCD_shiftr() - Shifts the entire display right                                        */
/*                                                                                       */
/* Shifts the display right one position without changing display RAM contents.          */
/* When the displayed data is shifted repeatedly, both lines move horizontally.          */
/* The second display line does not shift into the first display line.                   */
/*****************************************************************************************/
XStatus LCD_shiftr(void)
{
	XStatus sts;
	
	sts = LCD_docmd(LCDCC_SHIFTR, 0);
	
	return sts;
}
 
/*****************************************************************************************/
/* LCD_setcgadr() - Set the character generator RAM Address                              */
/*                                                                                       */
/* Sets the CG RAM address to the value in "addr".  The function                         */
/* also serves the purpose of telling the LCD controller that                            */
/* the character data should be written to the character generator RAM instead           */
/* of the data RAM.  The character generator RAM contains 8 user defined custom          */
/* characters                                                                            */
/*****************************************************************************************/
XStatus LCD_setcgadr(u32 addr)
{
	XStatus sts;
	
	sts = LCD_docmd(LCDCC_SETCGADR, (u8) addr);
	
	return sts;
}


/*****************************************************************************************/
/* LCD_setddadr() - Set the data RAM Address                                             */
/*                                                                                       */
/* Sets the data RAM address to the value in "addr".  The function                       */
/* also serves the purpose of telling the LCD controller that                            */
/* the character data should be written to the display RAM instead                       */
/* of the character generator RAM.                                                       */  
/*****************************************************************************************/

XStatus LCD_setddadr(u32 addr)
{
	XStatus sts;
	
	sts = LCD_docmd(LCDCC_SETDDADR, (u8) addr);
	
	return sts;
}

 
/*****************************************************************************************/
/* LCD_clrd() - Clear the display                                                        */
/*                                                                                       */
/* Writes blanks to the display and returns the cursor home.                             */
/*                                                                                       */
/*****************************************************************************************/
XStatus LCD_clrd(void)
{
	XStatus sts;
	
	sts = LCD_docmd(LCDCC_CLRD, 0);
	
	return sts;
}



// APIs for string related display functions

/*****************************************************************************************/
/* LCD_itoa() - converts an integer to ascii                                                 */
/*                                                                                       */
/* algorithm borrowed from ReactOS system libraries                                      */
/*                                                                                       */
/* converts the integer "value" to ascii in base "radix".  *string is a pointer to the   */
/* string that the result is returned to. Returns the pointer to the string.             */
/* Assumes that string[] is long enough to hold result.                                  */
/*****************************************************************************************/
char *LCD_itoa(int value, char *string, int radix)
{
	char tmp[33];
	char *tp = tmp;
	int i;
	u32 v;
	int sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return 0;
	}

	sign = (radix == 10 && value < 0);
	if (sign)
		v = -value;
	else
		v = (u32) value;
  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
			*tp++ = i+'0';
		else
			*tp++ = i + 'a' - 10;
	}
	sp = string;
	
	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;
  	return string;
}

 
/*****************************************************************************************/
/* LCD_wrstring() - Write a 0 terminated string to the LCD display                       */
/*                                                                                       */
/* Writes the null terminated string "s" to the LCD display                              */
/* starting at the current cursor position                                               */
/*****************************************************************************************/
XStatus LCD_wrstring(char* s)
{
	XStatus sts;
	int i=0;
	
 	while (s[i]!='\0')
 	{
		sts = LCD_wrchar(s[i]);
		if (sts != XST_SUCCESS)
			break;
		i++;
 	}
		
	return sts;
}
	

 
/*****************************************************************************************/
/* LCD_puthex() - Write a 32-bit unsigned hex number to LCD display                      */
/*                                                                                       */
/* Writes the 32-bit unsigned number "num" to the LCD display at the current             */
/* cursor position.                                                                      */
/*****************************************************************************************/

XStatus LCD_puthex(u32 num)
{
  char  buf[9];
  int   cnt;
  char  *ptr;
  int   digit;
  XStatus	sts;
  
  ptr = buf;
  for (cnt = 7 ; cnt >= 0 ; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;
    
    if (digit <= 9)
      *ptr++ = (char) ('0' + digit);
    else
      *ptr++ = (char) ('a' - 10 + digit);
  }

  *ptr = (char) 0;
  sts = LCD_wrstring(buf);
  return sts;
}


/*****************************************************************************************/
/* LCD_putnum() - Write a 32-bit number in Radix "radix" to LCD display                  */
/*                                                                                       */
/* Writes the 32-bit number "num" to the LCD display at the current                      */
/* cursor position. "radix" is the base to output the number in                          */
/*****************************************************************************************/
XStatus LCD_putnum(int num, int radix)
{
  char  buf[16];
  XStatus	sts;
  
  LCD_itoa(num, buf, radix);
  sts = LCD_wrstring(buf);
  return sts;
}
	

