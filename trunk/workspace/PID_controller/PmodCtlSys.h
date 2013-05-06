/*****************************************************************************
*
* Copyright Roy Kravitz, Portland State University 2013, 2014, 2015
*
* Filename:          PmodCtlSys.h
* Version:           1.00.a
* Description:       PmodCtlSys Driver Header File (API)
* Date:              Mon 22-Apr-13 (by Roy Kravitz)
*
* NOTE:  This driver assumes that a PmodCtlSys board is plugged into one of
* the Pmod expansion connectors on the Nexys 3 board.  There is no errot checking
* done by the driver to verify this, though
*****************************************************************************/

#ifndef PMODCTLSYS_H
#define PMODCTLSYS_H

/***************************** Include Files *******************************/
#include "xbasic_types.h"
#include "xstatus.h"
#include "xspi.h"

/************************** Constant Definitions ***************************/
#define PMODCTLSYS_ADC_ERROR	0xFFFF

/***************************** Function Prototypes *************************/

/*****
 * PmodCtlSys_init() - Initialize the PmodCtlsys
 * 
 * This function initializes the PmodCtlSys ADC (an MSP3202).  Sine
 * the MSP3202 is a SPI device the primary purpose of the function
 * is to configure and start the Xilinx SPI peripheral.  It checks
 * basic functionality of the SPI peripheral by running the self-test.
 * It also writes the correct data to the global send buffer PmodCtlSys_SndBuf[]
 * because its contents never change.  This code is based on the Xilinx SPI
 * driver "spi_polled_example.c" example included in the EDK.
 *
 *	@param	SpiInstancePtr is a pointer to the instance of the Xilinx SPI controller
 *			attached to the PmodCtlSys
 *
 *	@param	SpiDeviceID is the Device ID for the SPI controller instance attached
 *			to the PmodCtlSys
 *
 *	@return	XST_SUCCES if initialization is successful, XST_FAILURE otherwise
 *
 *****/
XStatus PmodCtlSys_init(XSpi *SpiInstancePtr, u16 SpiDeviceID);


/*****
 * PmodCtlSys_readADC() - read the ADC on the PmodCtlSys
 * 
 * This function performs a SPI read of the ADC on the PmodCtlSys.  It
 * returns a 12 bit ADC count giving the output voltage (e.g. the phototransistor)
 * of the control system.  This code is based on the Xilinx SPI
 * driver "spi_polled_example.c" example included in the EDK.  Uses the global
 * PmodCtlSys send and receive buffers.  the send buffer is configured by
 * PmodCtlSys_init() and does not change.
 *
 * NOTE:  The functions returns an ADC Count of PMODCTLSYS_ADC_ERROR (an illegal
 * count in a 12-bit ADC) if the SPI transfer fails
 *
 *	@param	SpiInstancePtr is a pointer to the SPI instance for the SPI controller
 *			attached to the PmodCtlSys
 *
 *	@return	Returns a 12-bit unsigned number representing the voltage
 *			on the output of the phototransistor if successful.  Returns
 *			PMODTCLTYS_ADC_ERROR if the operation fails
 *
 */
 u16 PmodCtlSys_readADC(XSpi *SpiInstancePtr);
 
 
 /*****
 * PmodCtlSys_ADCVolts() - converts a PmodCtlSys ADC count to volts
 * 
 * This function converts an ADC count from the PmodCtlSys to volts using the
 * following formula from the MSP3202 datasheet:
 *		Vin = (adc_count * VDD) / 4096  where VDD = 3.3v
 *
 *	@param	adc_cunt is the the 12-bit count from the PmodCtlSys ADC
 *
 *	@return	returns a floating point number reperesenting the voltage
 *			of the adc_count.
 *
 *****/
Xfloat32 PmodCtlSys_ADCVolts(u16 adc_count);

#endif /** PMODCTLSYS_H */
