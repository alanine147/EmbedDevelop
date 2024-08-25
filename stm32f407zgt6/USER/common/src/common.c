/*****************************************************************************/
/**
*
* @file cdd_rb.c
*
* This is the file which contains code for ring buffer complex driver.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date        Changes
* ----- ---- -------- -------------------------------------------------------
* 1.00  djs 2022/07/15 Initial release
*
* </pre>
*
* @note
*
******************************************************************************/

/***************************** Include Files *********************************/
#include "common.h"
/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/



/************************** Constant Definitions *****************************/


/************************** Variable Definitions *****************************/

void DelayWait(uint32 cnt)
{
    while (cnt > 0)
    {
        cnt--;
    }    
}
