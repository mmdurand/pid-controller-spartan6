/*****************************************************************************
* Filename:          C:\PSU_Projects\Nexys3_Development\Nexys3_Repository\MyProcessorIPLib/drivers/n3eif_v1_00_a/src/n3eif_selftest.c
* Version:           1.00.a
* Description:       Contains a diagnostic self-test function for the n3eif driver
* Date:              Thu Mar 14 06:09:59 2013 (by Create and Import Peripheral Wizard)
*****************************************************************************/


/***************************** Include Files *******************************/

#include "n3eif.h"

/************************** Constant Definitions ***************************/


/************************** Variable Definitions ****************************/


/************************** Function Definitions ***************************/

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
XStatus N3EIF_SelfTest(u32 baseaddr_p)
{
  int     Index;
  Xuint32 baseaddr;
  Xuint8  Reg8Value;
  Xuint16 Reg16Value;
  Xuint32 Reg32Value;
  
  /*
   * Check and get the device address
   */
  /*
   * Base Address maybe 0. Up to developer to uncomment line below.
  XASSERT_NONVOID(baseaddr_p != XNULL);
   */
  baseaddr = (Xuint32) baseaddr_p;

  xil_printf("********************************************\n\r");
  xil_printf("Nexys 3 Extended I/O Interface self-test...\n\r");
  xil_printf("*********************************************\n\n\r");

  /*
   * Write to user logic slave module register(s) and read back
   * NOTE:  This isn't much of a self-test because there is only
   * one register (the spare register) that isn't either controlled
   * by the hardware or controls the hardware.  Still, this single
   * write will establish that we can read/write the peripheral registers.
   */
  xil_printf("   - write 0x5A5A5A5A to SPARE\n\r");
  N3EIF_mWriteSPARE(baseaddr, 0x5A5A5A5A);
  Reg32Value = N3EIF_mReadSPARE(baseaddr);
  xil_printf("   - read %d from SPARE\n\r", Reg32Value);
  if ( Reg32Value != (Xuint32) 0x5A5A5A5A )
  {
    xil_printf("   - SPARE write/read failed\n\r");
    return XST_FAILURE;
  }
  
  xil_printf("   - write 0xA5A5A5A5 to SPARE\n\r");
  N3EIF_mWriteSPARE(baseaddr, 0xA5A5A5A);
  Reg32Value = N3EIF_mReadSPARE(baseaddr);
  xil_printf("   - read %d from SPARE\n\r", Reg32Value);
  if ( Reg32Value != (Xuint32) 0xA5A5A5A )
  {
    xil_printf("   - SPARE write/read failed\n\r");
    return XST_FAILURE;
  }
  
  xil_printf("   - Nexys 3 Extended I/O peripheral write/read passed\n\n\r");
  return XST_SUCCESS;
}
