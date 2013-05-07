/*****
 * project2_starter.c - ECE544-Project 2 - Control System Starter Application
 * Copyright Roy Kravitz, 2013, 2014
 *
 * 
 * Author:	Roy Kravitz
 * Version:	5.0
 * Date:	22-Apr-2013
 *
 * Revision History
 * ================
 * 23-Apr-09	RK		Created the first version
 * 04-May-09	RK		Add support for DCache and ICache
 * 18-Jan-10	RK		Adopted Tom Feliz' PID algorithm, added support for SerialCharter
 * 29-Jan-11	RK		Minor modifications for Winter 2011 term
 * 24-Jan-12	RK		Minor modifications for Winter 2012 term 
 * 22-Apr-13	RK		Ported to Nexys 3 and PmodCtlSys Rev 1.0
 *
 *
 * Description:
 * ============
 * This program implements a control application for the PmodCtlSys control system
 * used in Project 2.  The program uses a Xilinx timer/counter module in PWM mode
 * and a PmodCtlSys.  The human interface is provided by a PmodCLP (2 x 16 display) and
 * a PmodENC (rotary encoder) and the buttons, switches and LEDs on the Digilent Nexys 3
 *		sw[1:0] = 00:		Implements Bang-Bang (BB)control.  The rotary encoder is used to dial in
 *							the setpoint.  
 *
 *`		sw[1:0] = 01:		Implements Proportional-Integral-Derivitive (PID) control.  The rotary encoder is used
 *							to dial in the setpoint. Pressing BTN_NORTH selects between adjusting the Offset,
 *							Proportional Gain (GP), Derivitive Gain (GD), and Integral Gain (GI).  BTN_WEST and
 *							BTN_EAST are used to increase (BTN_EAST) or decrease (BTN_WEST) the offset and gain
 *							setting.  The user can simulate Proportional (P), and Proportional-Integral (PI) 
 *							control by setting the appropriate gains to 0.
 
 *		sw[1:0] = 10:		**RESERVED**
 *
 *		sw[1:0] = 11:		Characterizes the response to the system by applying 16 equally spaced
 * 							control over the active range of the ADC, allowing the output to settle, and
 * 							measuring the ADC value
 *
 * All of the tests are started by pressing the rotary encoder pushbutton.  Once pressed the button should
 * be held down until LED0 (rightmost) turns off indicating that the data is collected.  The data is sent
 * to STDOUT (normally the USB_Uart1 on the Nexys 3) when the button is released.   Use hyperterm or another terminal
 * emulator or the SerialCharter program to capture the data for analysis.
 *
 * sw[3] (leftmost) can be used to toggle the initial control state from PWM full-on (up - on) to PWM
 * full-off (down-off).
 *		
 *	NOTE:	The PmodCtlSys is self-contained with an on-board control circuit, anti-aliasing filter and
 *  Microchip MSP3202 12-bit SAR ADC.   The PmodCtlSys connects into any of the 12-pin Pmod expansion
 *	connectors on the board.  
 *
 *	THIS IS A STARTER APPLICATION THAT YOU CAN USE TO CREATE YOUR PROJECT 2 APPLICATION.  AS
 *	A RESULT, IT DOESN'T DO MUCH OTHER THAN LET YOU SELECT A TEST AND CHARACTERIZE YOUR
 *	HARDWARE
 *****/

/********** Include Files ***********/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "xparameters.h"
#include "xbasic_types.h"
#include "xtmrctr.h"
#include "xintc.h"
#include "xgpio.h"
#include "mb_interface.h"
#include "n3eif.h"
#include "PmodCtlSys.h"
#include "pwm_tmrctr.h"

/************************** Macros and Constants ******************************/

// Microblaze Cache Parameters
#define	USE_ICACHE				XPAR_MICROBLAZE_0_USE_ICACHE
#define	USE_DCACHE				XPAR_MICROBLAZE_0_USE_DCACHE
#define USE_DCACHE_WRITEBACK	XPAR_MICROBLAZE_DCACHE_USE_WRITEBACK

// GPIO, SPI and N3EIF parameters
#define GPIO_DEVICE_ID			XPAR_XPS_GPIO_0_DEVICE_ID
#define GPIO_BASEADDR			XPAR_XPS_GPIO_0_BASEADDR
#define GPIO_HIGHADDR			XPAR_XPS_GPIO_0_HIGHADDR
#define GPIO_OUTPUT_CHANNEL		1

#define SPI_DEVICEID			XPAR_XPS_SPI_0_DEVICE_ID
#define SPI_BASEADDR			XPAR_XPS_SPI_0_BASEADDR
#define SPI_HIGHADDR			XPAR_XPS_SPI_0_HIGHADDR

#define N3EIF_DEVICEID			XPAR_N3EIF_0_DEVICE_ID
#define N3EIF_BASEADDR			XPAR_N3EIF_0_BASEADDR
#define N3EIF_HIGHADDR			XPAR_N3EIF_0_HIGHADDR

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_XPS_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_XPS_INTC_0_FIT_TIMER_0_INTERRUPT_INTR
#define TIMER_INTERRUPT_ID		XPAR_XPS_INTC_0_XPS_TIMER_0_INTERRUPT_INTR

// Fixed Interval timer - 66.67MHz input clock, 5KHz output clock
// 1 msec time interval for FIT interrupt handler
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	XPAR_PROC_BUS_0_FREQ_HZ
#define FIT_COUNT				13333
#define FIT_CLOCK_FREQ_HZ		(FIT_IN_CLOCK_FREQ_HZ / FIT_COUNT)
#define FIT_COUNT_1MSEC			(FIT_CLOCK_FREQ_HZ / 1000)		

// PWM timer parameters
// Set PWM frequency = 10KHz
// Set PWM min and max duty cycles for keeping the PWM count in range
// and for setting the full-on and full-off duty cycles for Bang-bang control

#define PWM_TIMER_DEVICE_ID		XPAR_XPS_TIMER_0_DEVICE_ID
#define PWM_TIMER_BASEADDR		XPAR_XPS_TIMER_0_BASEADDR
#define PWM_TIMER_HIGHADDR		XPAR_XPS_TIMER_0_HIGHADDR
#define PWM_FREQUENCY_HZ		10000	
#define PWM_STEPDC_MIN			1
#define PWM_STEPDC_MAX			99

// ADC sample settings		
#define NUM_ADC_SAMPLES			250	

// Pushbutton and switch masks
#define	BTN_GAIN_SELECT			msk_BTN_NORTH
#define	BTN_GAIN_INCREASE		msk_BTN_EAST
#define BTN_GAIN_DECREASE		msk_BTN_WEST
#define	BTN_RUN_XFER			msk_BTN_ROT
#define BTN_MSK_ALLBUTTONS		(msk_BTN_EAST | msk_BTN_WEST | msk_BTN_NORTH | msk_BTN_ROT)

#define SW_INIT_HILO			msk_SWITCH3
#define SW_TEST_SELECT			(msk_SWITCH1 | msk_SWITCH0)	

// Rotary count parameters
#define ROTCNT_SPEEDUP_GATE		5
#define ROTCNT_SPEEDUP_FACTOR	10
#define	ROTCNT_DELTA			10	

// LED Masks
// LED[7:0] display is <PWM> <> <HILO><TEST1:0> <> <XFER> <RUN>
#define	LEDS_PWM					0x80
#define LEDS_HILO					0x20
#define	LEDS_TEST					0x18
#define	LEDS_XFER					0x02
#define LEDS_RUN					0x01
#define LEDS_ALLOFF					0x00
#define LEDS_ALLON					0xFF

// Cursor Display command bytes (used w/ LCD_docmd() and LCD_setcursor
// LCD_CURSOR_ON turns on a blinking cursor.  LCD_CURSOR_OFF turns it off
// LCD_DISPLAYONOFF is a command recognized by the xps_s3eif peripheral but
// not directly supported by a driver API call.  See the The S3E Starter Board 
// Peripheral Interface User Guide for details on all of the commands supported
// by the peripheral
#define LCD_DISPLAYONOFF			0x09
#define	LCD_CURSOR_ON				0x0E
#define	LCD_CURSOR_OFF				0x0C

// macro functions
#define MIN(a, b)  ( ((a) <= (b)) ? (a) : (b) )
#define MAX(a, b)  ( ((a) >= (b)) ? (a) : (b) )

/*****************************************************************************/

/****************************** typedefs and structures **********************/
typedef enum {
	TEST_BB = 0x0,
	TEST_PID = 0x01,
	TEST_RESERVED = 0x02,
	TEST_CHARACTERIZE = 0x03,
	TEST_INVALID = 0xFF
} Test_t;

typedef enum {
	SLCT_OFFSET, SLCT_GP, SLCT_GD, SLCT_GI, SLCT_INVALID
} ParamSel_t;

/*****************************************************************************/

/************************** Global Variables *********************************/
// Microblaze peripheral instances
XIntc IntrptCtlrInst; // Interrupt Controller instance
XTmrCtr PWMTimerInst; // PWM timer instance
XGpio GPIOInst; // GPIO instance
XSpi SPIInst; // SPI controller instance

// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler
volatile unsigned long timestamp; // timestamp since the program began
volatile Xuint32 gpio_port = 0; // GPIO port register - maintained in program

// The following variables are shared between the functions in the program
// such that they must be global
u16 sample[NUM_ADC_SAMPLES]; // sample array
int smpl_idx; // index into sample array
int adc_smple_interval; // approximate ADC sample interval

// Limits set by Characterization Function (unscaled)
int ADC_min_cnt, ADC_max_cnt; // Min and Max ADC counts
int PWM_min_cnt, PWM_max_cnt; // Min and Max PWM counts
Xfloat32 ADC_min_vout, ADC_max_vout; // Min and Max ADC voltage

// Control Parameters
int setpoint; // Control system setpoint (in ADC counts)
int offset; // Offset (in ADC counts)
int GP, GD, GI; // Gains - Proportional, Derivitive, Integral
bool init_high; // true if inital state of test should be full-on, false otherwise

int pwm_freq; // PWM frequency
int pwm_duty; // PWM duty cycle (for BB control)
int pwm_count; // PWM count (for PID control)


//PID debug info
struct {
	Xfloat32 err;
	Xfloat32 vdiff;
	int pwm_count;
} PIDdebug[NUM_ADC_SAMPLES];

/*---------------------------------------------------------------------------*/
int debugen = 0; // debug level/flag
/*---------------------------------------------------------------------------*/

/*****************************************************************************/

/************************** Function Prototypes ******************************/
int DoTest_BB(void); // Perform Bang Bang control test
int DoTest_PID(void); // Perform PID (and variations) test
int DoTest_Characterize(void); // Perform Characterization test

XStatus do_init(void); // initialize system
void delay_msecs(Xuint32 msecs); // busy-wait delay for "msecs" miliseconds
void voltstostrng(Xfloat32 v, char* s); // converts floating point volts to a 5 character formatted string

void FIT_Handler(void); // fixed interval timer interrupt handler
/*****************************************************************************/

/*********************************************/
/*              Main Program                 */
/*********************************************/
int main() {
	XStatus Status;
	u32 btnsw = 0x00000000, old_btnsw = 0x000000FF, diff_btnsw;
	u32 leds;
	int rotcnt, old_rotcnt;
	Test_t test;
	ParamSel_t param_select;

	bool wrlcd_dynamic;
	bool wrlcd_static;

	// initialize devices and set up interrupts, etc.
	Status = do_init();
	if (Status != XST_SUCCESS) {
		LCD_setcursor(1, 0);
		LCD_wrstring("****** ERROR *******");
		LCD_setcursor(2, 0);
		LCD_wrstring("INIT FAILED- EXITING");
		exit(XST_FAILURE);
	}

	// initialize the global variables
	timestamp = 0;
	pwm_freq = PWM_FREQUENCY_HZ;
	pwm_duty = PWM_STEPDC_MIN;

	// initialize the local variables
	btnsw = 0x00;
	old_btnsw = 0x00;
	rotcnt = 0;
	old_rotcnt = 0;
	param_select = SLCT_GP;
	wrlcd_dynamic = true;
	wrlcd_static = true;
	leds = LEDS_ALLOFF;

	// Enable the Microblaze caches and
	// kick off the processing by enabling the Microblaze interrupt
	// this starts the FIT timer which updates the timestamp.
	if (USE_ICACHE == 1) {
		microblaze_invalidate_icache();
		microblaze_enable_icache();
	}
	if (USE_DCACHE == 1) {
		microblaze_invalidate_dcache();
		microblaze_enable_dcache();
	}
	microblaze_enable_interrupts();

	// display the greeting
	LCD_setcursor(1, 0);
	LCD_wrstring("ECE544 Project 2");
	LCD_setcursor(2, 0);
	LCD_wrstring("Starter App-R5.0");
	NX3_writeleds(LEDS_ALLON);
	delay_msecs(2000);
	NX3_writeleds(LEDS_ALLOFF);

	// Set the PWM and ADC min and max counts by forcing a characterization
	LCD_clrd();
	LCD_setcursor(1, 0);
	LCD_wrstring("Characterizing..");
	DoTest_Characterize();
	LCD_setcursor(2, 0);
	LCD_wrstring("...Done         ");

	//*****GAIN AND OFFSET ARE HARDWIRED IN THE STARTER APP *****
	// initialize the control parameters
	// start the set point in the middle of the range	
	setpoint = (ADC_max_cnt - ADC_min_cnt) / 2;
	offset = 100;
	GP = 20;
	GD = 10;
	GI = 5;
	init_high = false;
	//*****GAIN AND OFFSET ARE HARDWIRED IN THE STARTER APP *****


	// main loop - there is no exit except by hardware reset
	while (1) {
		// write static portion of output to display
		if (wrlcd_static) {
			LCD_clrd();
			LCD_setcursor(1, 0);
			LCD_wrstring("G|Pxxx Ixxx Dxxx");
			LCD_setcursor(2, 0);
			LCD_wrstring("SP:+x.xx OFF:xxx");
			wrlcd_static = false;
		}

		// write the dynamic portion of output to display
		if (wrlcd_dynamic) {
			Xfloat32 v;
			u16 count;
			char s[20];
			u32 row, col;

			// display GP, GI, and GD
			LCD_setcursor(1, 3);
			LCD_wrstring("   ");
			LCD_setcursor(1, 3);
			LCD_putnum(GP, 10);

			LCD_setcursor(1, 8);
			LCD_wrstring("   ");
			LCD_setcursor(1, 8);
			LCD_putnum(GI, 10);

			LCD_setcursor(1, 13);
			LCD_wrstring("   ");
			LCD_setcursor(1, 13);
			LCD_putnum(GD, 10);

			LCD_setcursor(2, 13);
			LCD_wrstring("   ");
			LCD_setcursor(2, 13);
			LCD_putnum(offset, 10);

			// display the setpoint in volts
			count = setpoint;
			v = PmodCtlSys_ADCVolts(count);
			voltstostrng(v, s);
			LCD_setcursor(2, 3);
			LCD_wrstring("     ");
			LCD_setcursor(2, 3);
			LCD_wrstring(s);

			// place the cursor under the currently selected parameter
			LCD_docmd(LCD_DISPLAYONOFF, LCD_CURSOR_OFF);
			switch (param_select) {
			case SLCT_OFFSET:
				row = 2;
				col = 13;
				break;
			case SLCT_GP:
				row = 1;
				col = 3;
				break;
			case SLCT_GD:
				row = 1;
				col = 13;
				break;
			case SLCT_GI:
				row = 1;
				col = 8;
				break;
			case SLCT_INVALID:
				break;
			}
			if (param_select != SLCT_INVALID) {
				LCD_setcursor(row, col);
				LCD_docmd(LCD_DISPLAYONOFF, LCD_CURSOR_ON);
			}
			wrlcd_dynamic = false;
		}

		// read switches and buttons to get the test to perform and its initial value
		// display the selected test on the LEDs   
		NX3_readBtnSw(&btnsw);
		init_high = (btnsw & SW_INIT_HILO) ? true : false;
		test = btnsw & SW_TEST_SELECT;

		//update the HI/LO and TEST LEDs
		leds &= ~(LEDS_HILO | LEDS_TEST);
		leds |= (init_high == true) ? LEDS_HILO : 0;
		leds |= test << 3;
		NX3_writeleds(leds);

		// read rotary count and handle setpoint changes
		// accelerate setpoint if speedup threshold has been exceeded
		// Use MIN and MAX to keep scaled ADC count within range
		ROT_readRotcnt(&rotcnt);
		if (rotcnt != old_rotcnt) {
			if (abs(rotcnt - old_rotcnt) > ROTCNT_SPEEDUP_GATE) {
				setpoint += (rotcnt - old_rotcnt) * ROTCNT_DELTA
						* ROTCNT_SPEEDUP_FACTOR;
			} else {
				setpoint += (rotcnt - old_rotcnt) * ROTCNT_DELTA;
			}
			setpoint = MAX(ADC_min_cnt, MIN(setpoint, ADC_max_cnt));
			old_rotcnt = rotcnt;
			wrlcd_dynamic = true;
		} // rotary count changed

		//***** NOTE:  User interface handling should go in this section.  *****
		//***** The starter app just lets the user select the setpoint and *****
		//***** run the test selected by the switches                      *****

		// look at the buttons and take action on the rising edge of a button press
		btnsw &= BTN_MSK_ALLBUTTONS;
		diff_btnsw = (btnsw ^ old_btnsw) & btnsw;
		old_btnsw = btnsw;

		if (diff_btnsw & BTN_RUN_XFER) // run a test
		{
			Xfloat32 vADC, vSP; // VADC from circuit and Vsetpoint
			char s1[20], s2[30]; // display and print strings
			int (*funcPtr)(void); // pointer to test function
			int n; // number of samples to transfer (returned by test functions)

			switch (test) // set up for the selected test
			{
			case TEST_BB: // Bit Bang control
				strcpy(s1, "|BB  |Press RBtn");
				strcpy(s2, "Bit Bang Control Test Data");
				funcPtr = DoTest_BB;
				break;
			case TEST_PID: // PID control
				strcpy(s1, "|PID |Press RBtn");
				strcpy(s2, "PID Control Test Data");
				funcPtr = DoTest_PID;
				break;
			case TEST_CHARACTERIZE: // characterize control system simulator
				strcpy(s1, "|CHAR|Press RBtn");
				strcpy(s2, "Characterization Test Data");
				funcPtr = DoTest_Characterize;
				break;
			default: // no test to run
				strcpy(s1, "");
				strcpy(s2, "");
				funcPtr = NULL;
				break;
			}

			// Change the display to indicate that a test will be run			
			LCD_clrd();
			LCD_setcursor(1, 0);
			LCD_wrstring(s1);
			LCD_setcursor(2, 0);
			LCD_wrstring("LED OFF-Release ");

			// turn on Run LED to show the test has begun
			// and do the test.  The test will return when the ADC samples array
			// has been filled.  Turn off the rightmost LED after the data has been 
			// captured to let the user know he/she can release the button
			if (funcPtr != NULL) {
				leds |= LEDS_RUN;
				NX3_writeleds(leds);
				n = funcPtr();
				leds &= ~LEDS_RUN;
				NX3_writeleds(leds);
				// wait for the Rotary Encoder button to be released
				// and then send the sample data to stdout
				do {
					NX3_readBtnSw(&btnsw);
					delay_msecs(10);
				} while (btnsw & BTN_RUN_XFER);

				// turn on Transfer LED to indicate that data is being transmitted
				// Show the transfer  traffic on the LCD
				leds |= LEDS_XFER;
				NX3_writeleds(leds);
				LCD_clrd();
				LCD_setcursor(1, 0);
				LCD_wrstring("Sending Data....");
				LCD_setcursor(2, 0);
				LCD_wrstring("S:    DATA:     ");

				// transfer the descriptive heading followed by the data	
				xil_printf("\n\r%s\tAppx. Sample Interval: %d msec\n\r", s2,
						adc_smple_interval);
				xil_printf("sample\tADCCount\tVout\tsetpoint\tPWMCount\n\r");

				// tell SerialCharter to start gathering data)
				xil_printf("===STARTPLOT===\n");

				for (smpl_idx = 2; smpl_idx < n; smpl_idx++) {
					u16 count;

					count = sample[smpl_idx];
					vADC = PmodCtlSys_ADCVolts(count);
					vSP = PmodCtlSys_ADCVolts(setpoint);
					voltstostrng(vADC, s1);
					voltstostrng(vSP, s2);
					xil_printf("%d\t%d\t%s\t%s\t%d\n\r", smpl_idx, count, s1,
							s2, PIDdebug[smpl_idx].pwm_count);
					LCD_setcursor(2, 2);
					LCD_wrstring("   ");
					LCD_setcursor(2, 2);
					LCD_putnum(smpl_idx, 10);
					LCD_setcursor(2, 11);
					LCD_wrstring("     ");
					LCD_setcursor(2, 11);
					LCD_putnum(count, 10);
				}

				// tell SeriaCharter to stop gathering data and display the graph			
				xil_printf("===ENDPLOT===\n");

				// transfer complete - turn Transfer LED off and wrap up command processing
				leds &= ~LEDS_XFER;
				NX3_writeleds(leds);
				wrlcd_static = true;
				wrlcd_dynamic = true;
			}
		} // run a test

		// wait a bit and start again
		delay_msecs(100);
	} // while(1) loop
} // end main()


/*********************************************/
/*             Test Functions                */
/*********************************************/

/*****
 * DoTest_BB() - Perform Bang-Bang control
 * 
 * This function adjusts the global "pwm_duty" value to try to bring the
 * control system simulator closer to the ADC setpoint.  The initial state of the PWM
 * is derived from the global "init_high" flag. NUM_ADC_SAMPLES are collected
 * into the global array sample[] and the count returned.  The function toggles the TEST_RUNNING
 * signal high for the duration of the test as a debug aid.  An approximate
 * ADC sample interval is written to the global variable "adc_smpl_interval"
 *
 * Bang-bang control is the simplest form of closed loop control.  The PWM is turned either full-on
 * of full-off depending on whether the output is below or above the setpoint. 
 *
 * NOTE:  THIS ALGORIHM IS LEFT AS AN EXERCISE FOR THE PROGRAMMER
 *****/
int DoTest_BB(void)
{

	XStatus Status;         // Xilinx return status
	unsigned tss;           // starting timestamp
	u16 adc_1, adc_2; 		// holds the ADC count - sampled twice and averaged
	u16 adc_cnt; 			// ADC counts to display
	int n; 					// number of samples
    Xfloat32 setpoint_volts, adc_reading_volts;

	//The initial state of the PWM is derived from the global "init_high" flag.
	// true if inital state of test should be full-on, false otherwise
	if (init_high == true)
		pwm_duty = PWM_STEPDC_MAX;
	else
		pwm_duty = PWM_STEPDC_MIN;

	//Checks Status
	Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
	if (Status == XST_SUCCESS) {
		PWM_Start(&PWMTimerInst);
	} else {
		return -1;
	}
	delay_msecs(100);

	smpl_idx = 0;
	n = 0;
	tss = timestamp;

	//Calculate in volts our setpoint
	setpoint_volts = PmodCtlSys_ADCVolts(setpoint);
	////-------------------------------
	LCD_clrd();
	LCD_setcursor(1, 0);
	LCD_wrstring("BANG BANG");

	while (smpl_idx <= NUM_ADC_SAMPLES)
	{
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
		if (Status == XST_SUCCESS)
		{
			PWM_Start(&PWMTimerInst);
		}
		else {
			return -1;
		}

		//Sample the ADC and average the readings
		adc_1 = PmodCtlSys_readADC(&SPIInst);
		delay_msecs(1);
		adc_2 = PmodCtlSys_readADC(&SPIInst);
		adc_cnt = (adc_1 + adc_2) / 2;

		//Converting the ADC average into volts
		adc_reading_volts = PmodCtlSys_ADCVolts(adc_cnt);

		if (adc_reading_volts < setpoint_volts)
			pwm_duty = PWM_STEPDC_MIN; //input to 100% duty cycle
		else
			pwm_duty = PWM_STEPDC_MAX;

		//CHECK STATUS
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
		if (Status == XST_SUCCESS) {
			PWM_Start(&PWMTimerInst);
		} else {
			return -1;
		}

		//store in array
		sample[smpl_idx++] = adc_cnt;
		n++;
	}
	adc_smple_interval = (timestamp - tss) / smpl_idx;
	return n;

}

/*****
 * DoTest_PID() - Perform PID control and it's derivitives
 * 
 * This function adjusts the global "pwm_duty", "offset" and gains (GP, GI, GD) values to try to 
 * bring the control system simulator close to the ADC setpoint.  The initial state of the PWM
 * is derived from the global "init_high" flag. NUM_ADC_SAMPLES are collected
 * into the global array sample[] and the count returned.  The function toggles the TEST_RUNNING
 * signal high for the duration of the test as a debug aid.  An approximate
 * ADC sample interval is written to the global variable "adc_smpl_interval"
 *
 * PID control provides finer control over the control input than that offered by Bang-bang control.
 * The biggest contributor to the control input is a value proportional to the distance between the
 * setpoint and the actual point.  Integral and Derivitive terms based on the history of the control
 * system are used to fine tune the control input.
 *
 * NOTE:  THIS ALGORIHM IS LEFT AS AN EXERCISE TO THE READER 
 *****/
int DoTest_PID(void)
{
	return 0;
}

/*****
 * DoTest_Characterize() - Perform the Characterization test
 * 
 * This function starts the duty cycle at the minimum duty cycle and
 * then increases it to the max duty cycle for the test.
 * Samples are collected into the global array sample[].  
 * The function toggles the TEST_RUNNING signal high for the duration
 * of the test as a debug aid and adjusts the global "pwm_duty"
 *
 * The test also sets the global ADC min and max counts to
 * help limit the ADC counts to the active range for the circuit
 *****/
int DoTest_Characterize(void) {
	XStatus Status; // Xilinx return status
	unsigned tss; // starting timestamp
	u16 adc_1, adc_2; // holds the ADC count - sampled twice and averaged
	u16 adc_cnt; // ADC counts to display
	int n; // number of samples
	Xuint32 freq, dutyfactor; // current frequency and duty factor


	// stabilize the PWM output (and thus the lamp intensity) at the
	// minimum before starting the test
	pwm_duty = PWM_STEPDC_MIN;
	Status = PWM_SetParams(&PWMTimerInst, pwm_freq, pwm_duty);
	if (Status == XST_SUCCESS) {
		PWM_Start(&PWMTimerInst);
	} else {
		return -1;
	}
	delay_msecs(1000);

	// sweep the duty cycle from STEPDC_MIN to STEPDC_MAX
	smpl_idx = PWM_STEPDC_MIN;
	n = 0;
	tss = timestamp;
	while (smpl_idx <= PWM_STEPDC_MAX)
	{
		Status = PWM_SetParams(&PWMTimerInst, pwm_freq, smpl_idx);
		if (Status == XST_SUCCESS)
		{
			PWM_Start(&PWMTimerInst);
		} else {
			return -1;
		}
		// sample the ADC twice and average the readings
		adc_1 = PmodCtlSys_readADC(&SPIInst);
		delay_msecs(1);
		adc_2 = PmodCtlSys_readADC(&SPIInst);
		adc_cnt = (adc_1 + adc_2) / 2;
		sample[smpl_idx++] = adc_cnt;

		n++;
		delay_msecs(20);
	}
	adc_smple_interval = (timestamp - tss) / smpl_idx;

	// initialize the ADC min and max counts from the saved data
	ADC_min_cnt = ADC_max_cnt = 0;
	for (n = PWM_STEPDC_MIN; n <= PWM_STEPDC_MAX; n++) {
		if (sample[n] < ADC_min_cnt)
			ADC_min_cnt = sample[n];

		if (sample[n] > ADC_max_cnt)
			ADC_max_cnt = sample[n];
	}
	ADC_max_vout = PmodCtlSys_ADCVolts(ADC_min_cnt);
	ADC_min_vout = PmodCtlSys_ADCVolts(ADC_max_cnt);

	// initialize the PWM min and max counts - these are constants so can be calculated here
	// count = ((pwm duty cycle * PLB clock frequency[Hz]) / pwm frequency[Hz]) - 2
	PWM_GetParams(&PWMTimerInst, &freq, &dutyfactor);
	PWM_min_cnt = ((PWM_STEPDC_MIN * .01 * PLB_CLOCK_FREQ_HZ) / freq) - 2;
	PWM_max_cnt = ((PWM_STEPDC_MAX * .01 * PLB_CLOCK_FREQ_HZ) / freq) - 2;
	return n;
}

/*********************************************/
/*            Support Functions              */
/*********************************************/

/*****
 * do_init() - initialize the system
 * 
 * This function is executed once at start-up and after a reset.  It initializes
 * the peripherals and registers the interrupt handlers
 *****/
XStatus do_init(void) {
	XStatus Status; // status from Xilinx Lib calls

	// initialize the N3EIF driver
	// rotary encoder is set to increment from 0 by 1
	// NOTE: it's the relative change from the last time the rotary encoder was sampled
	// not the absolute change that matters.  The actual change is handled in main() so we
	// can apply a "speedup" factor if the knob is being turned quickly.
	N3EIF_init(N3EIF_BASEADDR);
	ROT_init(1, false);
	ROT_clear();

	// initialize the GPIO instance
	Status = XGpio_Initialize(&GPIOInst, GPIO_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	// GPIO channel 1 is an 8-bit output port that your application can
	// use.  None of the bits are used by this program
	XGpio_SetDataDirection(&GPIOInst, GPIO_OUTPUT_CHANNEL, 0xF0);
	XGpio_DiscreteWrite(&GPIOInst, GPIO_OUTPUT_CHANNEL, gpio_port);

	// initialize the PWM timer/counter instance but do not start it
	// do not enable PWM interrupts
	Status = PWM_Initialize(&PWMTimerInst, PWM_TIMER_DEVICE_ID, false);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// initialize the PmodCtlSys
	Status = PmodCtlSys_init(&SPIInst, SPI_DEVICEID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// initialize the interrupt controller
	Status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// connect the fixed interval timer (FIT) handler to the interrupt
	Status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
			(XInterruptHandler) FIT_Handler, (void *) 0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts, specifically real mode so that
	// the the  FIT can cause interrupts thru the interrupt controller.
	Status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}

/*****
 * delay_msecs() - delay execution for "n" msecs
 * 
 *		timing is approximate but we're not looking for precision here, just
 *		a uniform delay function.  The function uses the global "timestamp" which
 *		is incremented every msec by FIT_Handler().
 *
 * NOTE:  Assumes that this loop is running faster than the fit_interval ISR (every msec)
 *****/
void delay_msecs(Xuint32 msecs) {
	unsigned long target;

	if (msecs == 0) {
		return;
	}
	target = timestamp + msecs;
	while (timestamp != target) {
		// spin until delay is over
	}
	return;
}

/*****
 * voltstostrng() - converts volts to a fixed format string
 * 
 * accepts an Xfloat32 voltage reading and turns it into a 5 character string
 * of the following format:
 *		(+/-)x.yy 
 * where (+/-) is the sign, x is the integer part of the voltage and yy is
 * the decimal part of the voltage.
 *	
 * NOTE:  Assumes that s points to an array of at least 6 bytes.	
 *****/
void voltstostrng(Xfloat32 v, char* s) {
	Xfloat32 dpf, ipf;
	Xuint32 dpi;
	Xuint32 ones, tenths, hundredths;

	// form the fixed digits
	dpf = modff(v, &ipf);
	dpi = dpf * 100;
	ones = abs(ipf) + '0';
	tenths = (dpi / 10) + '0';
	hundredths = (dpi - ((tenths - '0') * 10)) + '0';

	// form the string and return
	*s++ = ipf == 0 ? ' ' : (ipf > 0 ? '+' : '-');
	*s++ = (char) ones;
	*s++ = '.';
	*s++ = (char) tenths;
	*s++ = (char) hundredths;
	*s = 0;
	return;
}

/*********************************************/
/*            Interrupt Handlers             */
/*********************************************/

/*****
 * FIT_Handler() - Fixed interval timer interrupt handler 
 *  
 * updates the global "timestamp" every milisecond.  "timestamp" is used for the delay_msecs() function
 * and as a time stamp for data collection and reporting
 *****/
void FIT_Handler(void) {
	static int ts_interval = 0; // interval counter for incrementing timestamp

	// update timestamp	
	ts_interval++;
	if (ts_interval > FIT_COUNT_1MSEC) {
		timestamp++;
		ts_interval = 1;
	}
}
