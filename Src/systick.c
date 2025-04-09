/* systick implementation */

#include "my_board_info.h"

#define MSECS_PER_SEC     1000
#define CCPMS             ((CLOCK_FREQ / MSECS_PER_SEC) - 1) // Clock Cycles per msec (clock starts at 0 so subtract 1 here)

/* Function to delay
 * @param msecs - time in millisecods to delay
 */
void systick_delay_msecs(int msecs) {
  // set the number of clock cycles per msec in the Reload Value REG                              e.g.
  SysTick->LOAD = CCPMS; // (SYST_RVR register - the clock registers are defined in the ARM Cortex-M7 Devices Generic User Guide)
                         // the constants for these are defined in core_cmX.h file which is in Drivers/CMSIS/Include for your board's firmware
  // clear the systick current value REG
  SysTick->VAL = 0x0; // (SYST_CVR register)

  // select the clock source & enable the systick - start counting down from CCPMS
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // (SYST_CSR register)

  for (int i=0; i<msecs; ++i) {
    // wait until count flag is set (happens when LOAD reg reaches 0)
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {}
    // the LOAD reg is now reloaded and the next countdown begins
  }

  // Requested delay time has expired. Disable systick and return
  SysTick->CTRL = 0x0;
}
