/*******************************************************************************
  MPLAB Harmony Touch Host Interface v1.1.0 Release

  Company:
    Microchip Technology Inc.

  File Name:
      touchExample.c

  Summary:
    This file contains the example "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>      // Defines NULL
#include <stdbool.h>     // Defines true
#include <stdlib.h>      // Defines EXIT_FAILURE
#include "definitions.h" // SYS function prototypes

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int touch_mainloop_example(void);

int touch_mainloop_example(void)
{
  /* Initialize all modules */
  SYS_Initialize(NULL);

  touchDeviceInit();
  touchTuneInit();
  // Read setup block
  communicationStatus.readStatus = ReadSetupBlock((uint8_t)sizeof(setup_block), (uint8_t *)&setup_block);

  /* TO DO : modify setup block parameters here
   * from default valus if required
   * For example: To set NTHR for Key 0 to 20
   * setup_block.KEY_0_NTHR = 20;
   */
  communicationStatus.writeStatus = WriteSetupBlock((uint8_t)sizeof(setup_block), (uint8_t *)&setup_block);
  while (true)
  {

    touchDeviceProcess();
    touchTuneProcess();
    /* If touch tune is enabled, then CHANGE pin may not be active as key status are read in tune continuously.
    In that case, refer to GUI to get the touch status */
    /* Read all status-bytes
     * if CHANGE pin is asserted */
  }

  /* Execution should not come here during normal operation */

  return (EXIT_FAILURE);
}

/*******************************************************************************
 End of File
*/
