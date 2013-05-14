/*****************************************************************************
*
* Copyright Roy Kravitz, Portland State University 2013, 2014, 2015
* Filename:          n3eif_l.h
* Version:           1.00.a
* Description:       n3eif Low Level Driver Header File
* Date:              Sun 17-Mar-13 (by Roy Kravitz)
*
* NOTE:  This driver assumes that a Digilent PmodCLP (2 x 16 LCD) is plugged into
* two (2) of the Pmod headers on the Nexys3 board and that a Digilent PmodENC
* is plugged into a third (1) Pmod header on the Nexys3.  There is no errot checking
* done by the driver to verify this, though
*****************************************************************************/

#ifndef N3EIF_L_H
#define N3EIF_L_H

/***************************** Include Files *******************************/

#include "xbasic_types.h"
#include "xstatus.h"
#include "xil_io.h"

/************************** Constant Definitions ***************************/

/**
 * Nexys 3 Interface Register offsets
 * -- BTNSW_IN 	 : debounced buttons and switches	(I)
 * -- ROTSTS	 : rotary encoder status register	(I)
 * -- ROTCNTL	 : rotary encoder count bits [7:0]	(I)
 * -- ROTCNTH	 : rotary encoder count bits[15:8]	(I)
 * -- LCDSTS	 : LCD Controller status			(I)
 * -- LEDS_DATA	 : LED register values				(O)
 * -- ROTCTL	 : rotary encoder control			(O)
 * -- LCDCMD	 : LCD controller command			(O)
 * -- LCDDATA	 : LCD controller data				(O)
 * -- SPARE		 : Spare Registers **RESERVED**		(I)
 */
 
#define N3EIF_USER_SLV_SPACE_OFFSET (0x00000000)
#define N3EIF_BTNSW_IN_OFFSET (N3EIF_USER_SLV_SPACE_OFFSET + 0x00000000)
#define N3EIF_ROTSTS_OFFSET (N3EIF_USER_SLV_SPACE_OFFSET + 0x00000004)
#define N3EIF_ROTCNTL_OFFSET (N3EIF_USER_SLV_SPACE_OFFSET + 0x00000008)
#define N3EIF_ROTCNTH_OFFSET (N3EIF_USER_SLV_SPACE_OFFSET + 0x0000000C)
#define N3EIF_LCDSTS_OFFSET (N3EIF_USER_SLV_SPACE_OFFSET + 0x00000010)
#define N3EIF_LEDS_DATA_OFFSET (N3EIF_USER_SLV_SPACE_OFFSET + 0x00000014)
#define N3EIF_ROTCTL_OFFSET (N3EIF_USER_SLV_SPACE_OFFSET + 0x00000018)
#define N3EIF_LCDCMD_OFFSET (N3EIF_USER_SLV_SPACE_OFFSET + 0x0000001C)
#define N3EIF_LCDDATA_OFFSET (N3EIF_USER_SLV_SPACE_OFFSET + 0x00000020)
#define N3EIF_SPARE_OFFSET (N3EIF_USER_SLV_SPACE_OFFSET + 0x00000024)


/**************************** Type Definitions *****************************/


/***************** Macros (Inline Functions) Definitions *******************/

/**
 *
 * Write a value to a N3EIF register. A 32 bit write is performed.
 * If the component is implemented in a smaller width, only the least
 * significant data is written.
 *
 * @param   BaseAddress is the base address of the N3EIF device.
 * @param   RegOffset is the register offset from the base to write to.
 * @param   Data is the data written to the register.
 *
 * @return  None.
 *
 * @note
 * C-style signature:
 * 	void N3EIF_mWriteReg(Xuint32 BaseAddress, unsigned RegOffset, Xuint32 Data)
 *
 */
#define N3EIF_mWriteReg(BaseAddress, RegOffset, Data) \
 	Xil_Out32((BaseAddress) + (RegOffset), (Xuint32)(Data))

/**
 *
 * Read a value from a N3EIF register. A 32 bit read is performed.
 * If the component is implemented in a smaller width, only the least
 * significant data is read from the register. The most significant data
 * will be read as 0.
 *
 * @param   BaseAddress is the base address of the N3EIF device.
 * @param   RegOffset is the register offset from the base to write to.
 *
 * @return  Data is the data from the register.
 *
 * @note
 * C-style signature:
 * 	Xuint32 N3EIF_mReadReg(Xuint32 BaseAddress, unsigned RegOffset)
 *
 */
#define N3EIF_mReadReg(BaseAddress, RegOffset) \
 	Xil_In32((BaseAddress) + (RegOffset))


/**
 *
 * Write/Read 32 bit value to/from N3EIF user logic slave registers.
 *
 * @param   BaseAddress is the base address of the N3EIF device.
 * @param   RegOffset is the offset from the slave register to write to or read from.
 * @param   Value is the data written to the register.
 *
 * @return  Data is the data from the user logic slave register.
 *
 * @note
 * C-style signature:
 * 	void N3EIF_mWriteSlaveRegn(Xuint32 BaseAddress, unsigned RegOffset, Xuint32 Value)
 * 	Xuint32 N3EIF_mReadSlaveRegn(Xuint32 BaseAddress, unsigned RegOffset)
 *
 */
#define N3EIF_mWriteBTNSW_IN(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (N3EIF_BTNSW_IN_OFFSET), (Xuint32)(Value))
#define N3EIF_mWriteROTSTS(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (N3EIF_ROTSTS_OFFSET), (Xuint32)(Value))
#define N3EIF_mWriteROTCNTL(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (N3EIF_ROTCNTL_OFFSET), (Xuint32)(Value))
#define N3EIF_mWriteROTCNTH(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (N3EIF_ROTCNTH_OFFSET), (Xuint32)(Value))
#define N3EIF_mWriteLCDSTS(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (N3EIF_LCDSTS_OFFSET), (Xuint32)(Value))
#define N3EIF_mWriteLEDS_DATA(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (N3EIF_LEDS_DATA_OFFSET), (Xuint32)(Value))
#define N3EIF_mWriteROTCTL(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (N3EIF_ROTCTL_OFFSET), (Xuint32)(Value))
#define N3EIF_mWriteLCDCMD(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (N3EIF_LCDCMD_OFFSET), (Xuint32)(Value))
#define N3EIF_mWriteLCDDATA(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (N3EIF_LCDDATA_OFFSET), (Xuint32)(Value))
#define N3EIF_mWriteSPARE(BaseAddress, Value) \
 	Xil_Out32((BaseAddress) + (N3EIF_SPARE_OFFSET), (Xuint32)(Value))

#define N3EIF_mReadBTNSW_IN(BaseAddress) \
 	Xil_In32((BaseAddress) + (N3EIF_BTNSW_IN_OFFSET))
#define N3EIF_mReadROTSTS(BaseAddress) \
 	Xil_In32((BaseAddress) + (N3EIF_ROTSTS_OFFSET))
#define N3EIF_mReadROTCNTL(BaseAddress) \
 	Xil_In32((BaseAddress) + (N3EIF_ROTCNTL_OFFSET))
#define N3EIF_mReadROTCNTH(BaseAddress) \
 	Xil_In32((BaseAddress) + (N3EIF_ROTCNTH_OFFSET))
#define N3EIF_mReadLCDSTS(BaseAddress) \
 	Xil_In32((BaseAddress) + (N3EIF_LCDSTS_OFFSET))
#define N3EIF_mReadLEDS_DATA(BaseAddress) \
 	Xil_In32((BaseAddress) + (N3EIF_LEDS_DATA_OFFSET))
#define N3EIF_mReadROTCTL(BaseAddress) \
 	Xil_In32((BaseAddress) + (N3EIF_ROTCTL_OFFSET))
#define N3EIF_mReadLCDCMD(BaseAddress) \
 	Xil_In32((BaseAddress) + (N3EIF_LCDCMD_OFFSET))
#define N3EIF_mReadLCDDATA(BaseAddress) \
 	Xil_In32((BaseAddress) + (N3EIF_LCDDATA_OFFSET))
#define N3EIF_mReadSPARE(BaseAddress) \
 	Xil_In32((BaseAddress) + (N3EIF_SPARE_OFFSET))

/************************** Function Prototypes ****************************/

/**
 *
 * Run a self-test on the driver/device. Note this may be a destructive test if
 * resets of the device are performed.
 *
 * If the hardware system is not built correctly, this function may never
 * return to the caller.
 *
 * @param   baseaddr_p is the base address of the N3EIF instance to be worked on.
 *
 * @return
 *
 *    - XST_SUCCESS   if all self-test code passed
 *    - XST_FAILURE   if any self-test code failed
 *
 * @note    Caching must be turned off for this function to work.
 * @note    Self test may fail if data memory and device are not on the same bus.
 *
 */
XStatus N3EIF_SelfTest(u32 baseaddr_p);


/**
 * N3E_usleep() - delay "usec" microseconds.  This function should be in libc but it seems
 * to be missing. 
 *
 * @param	usec is the number of microseconds to delay
 *
 * @return	NONE
 *
 * @note   This emulation assumes that the microblaze is running @ 100MHz and
 *			takes 20 clocks per iiteration - this is probably bogus
 *			but it's a start.
*/
void N3EIF_usleep(u32 usec);

#endif /** N3EIF_L_H */


