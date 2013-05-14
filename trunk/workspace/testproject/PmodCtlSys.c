/*****************************************************************************
*
* Copyright Roy Kravitz, Portland State University 2013, 2014, 2015
*
* Filename:          PmodCtlSys.c
* Version:           1.00.a
* Description:       PmodCtlSys Driver Source Code (API)
* Date:              Mon 22-Apr-13 (by Roy Kravitz)
*
* NOTE:  This driver assumes that a PmodCtlSys board is plugged into one of
* the Pmod expansion connectors on the Nexys 3 board.  There is no errot checking
* done by the driver to verify this, though
*****************************************************************************/


/***************************** Include Files *******************************/

#include "PmodCtlSys.h"

/************************** Constant Declarations ***************************/

// PmodCtlSys settings
// PmodCtlSys is the first device on the SPI bus
// MSP3202 command bit masks are also defined
// it takes 3 8-bit transfers to return a 12-bit count

#define PMODCTLSYS_SPI_SS		0x01
#define PMODCTLSYS_BUF_SIZE		3
#define PMODCTLSYS_ADC_ERROR	0xFFFF

#define MSP3202_START_MSK		0x01
#define MSP3202_SGL_MSK			0x00
#define MSP3202_DIFF_MSK		0x01
#define MSP3202_SEL_CHNL0_MSK	0x00
#define MSP3202_SEL_CHNL1_MSK	0x01
#define MSP3202_SEL_MSBF_MSK	0x01
#define MSP3202_SEL_LSB_MSK		0x00


/**************************** Global Variables *****************************/
// PmodCtlSys ADC buffers.  Make them global so we don't use a lot of stack space
static u8				PmodCtlSys_SndBuf[PMODCTLSYS_BUF_SIZE];
static u8				PmodCtlSys_RcvBuf[PMODCTLSYS_BUF_SIZE];


/************************** Function Definitions ***************************/

/*****
 * PmodCtlSys_init() - initialize the PmodCtlsys
 * 
 * This function initializes the PmodCtlSys ADC (an MSP3202).  Sine
 * the MSP3202 is a SPI device the primary purpose of the function
 * is to configure and start the Xilinx SPI peripheral.  It checks
 * basic functionality of the SPI peripheral by running the self-test.
 * It also writes the correct data to the global send buffer PmodCtlSys_SndBuf[]
 * because its contents never change.  This code is based on the Xilinx SPI
 * driver "spi_polled_example.c" example included in the EDK.
 *****/
XStatus PmodCtlSys_init(XSpi *SpiInstancePtr, u16 SpiDeviceID)
{
	XStatus Status;				// return status from SPI driver functions
	XSpi_Config *ConfigPtr;		// Pointer to Configuration data

	// Initialize the SPI driver so that it is  ready to use.
	ConfigPtr = XSpi_LookupConfig(SpiDeviceID);
	if (ConfigPtr == NULL)
	{
		return XST_DEVICE_NOT_FOUND;
	}
	Status = XSpi_CfgInitialize(SpiInstancePtr, ConfigPtr,
				  ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// Perform a self-test to ensure that the hardware was built correctly
	Status = XSpi_SelfTest(SpiInstancePtr);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// Set the Spi device as a master,
	// SS goes low for entire transaction (does not toggle every 8 bits)
	// All other bits are OK as defaults
	Status = XSpi_SetOptions(SpiInstancePtr, XSP_MASTER_OPTION | XSP_MANUAL_SSELECT_OPTION);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// Start the SPI driver so that the device is enabled
	// and then disable the Global interrupt because we
	// are using the peripheral in polled mode
	XSpi_Start(SpiInstancePtr);
	XSpi_IntrGlobalDisable(SpiInstancePtr);
	
	// initialize the SPI send buffer.  Since the PmodCtlSys only
	// uses the channel 0 ADC in the MPS3202 the send buffer
	// contents will always be the same.
	//	Command Byte 1 - Start bit is in the LSB, others are don't care
	//	Command Byte 2 - single-ended input, Select Channel 0, send data most-significant bit first	
	//	Command Byte 2 - needed to have ADC return all the counts bits.  All bits are don't care
	PmodCtlSys_SndBuf[0] = MSP3202_START_MSK;
	PmodCtlSys_SndBuf[1] = (MSP3202_SGL_MSK << 7) | (MSP3202_SEL_CHNL0_MSK << 6) | (MSP3202_SEL_MSBF_MSK << 5);
	PmodCtlSys_SndBuf[2] = 0x55;
	
	return XST_SUCCESS;
}


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
 *****/
u16 PmodCtlSys_readADC(XSpi *SpiInstancePtr)
{
	XStatus Status;				// return status from SPI driver functions
	u16 adc_cnt;				// The ADC count

	// Set the slave select mask.  This mask is used by the transfer command
	// to indicate which slave select line to enable
	Status = XSpi_SetSlaveSelect(SpiInstancePtr, PMODCTLSYS_SPI_SS);
	if (Status != XST_SUCCESS)
	{
		return PMODCTLSYS_ADC_ERROR;
	}

	// Transfer the command and receive the ADC count.  In polled mode
	// the transfer function blocks until the transfer is complete
	// We will send an receive all 3 bytes
	Status = XSpi_Transfer(SpiInstancePtr, PmodCtlSys_SndBuf, PmodCtlSys_RcvBuf, PMODCTLSYS_BUF_SIZE);
	if (Status != XST_SUCCESS)
	{
		return PMODCTLSYS_ADC_ERROR;
	}
		
	
	// SPI transfer was successful so we assume there is a valid ADC count in the buffer
	// The ADC count is returned in the following format:
	//      RcvBuf[0] = x   x   x   x   x   x   x   x
	//		RcvBuf[1] = x   x   x   0   B11 B10 B09 B08
	//		RcvBuf[2] = B07 B06 B05 B04 B03 B02 B01 B00
	adc_cnt = ((PmodCtlSys_RcvBuf[1] << 8) | (PmodCtlSys_RcvBuf[2] << 0)) & 0x0FFF;
	return adc_cnt;
}


/*****
 * PmodCtlSys_ADCVolts() - converts a PmodCtlSys ADC count to volts
 * 
 * This function converts an ADC count from the PmodCtlSys to volts using the
 * following formula from the MSP3202 datasheet:
 *		Vin = (adc_count * VDD) / 4096  where VDD = 3.3v
 *****/
Xfloat32 PmodCtlSys_ADCVolts(u16 adc_count)
{
Xfloat32 vin;

vin = (Xfloat32) (adc_count * 3.30) / 4096.0;
return vin;
}

