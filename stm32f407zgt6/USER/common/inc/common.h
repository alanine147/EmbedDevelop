/*****************************************************************************/
/**
*
* @file cdd_rb.h
*
* This is the file which contains header for log complex driver.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date        Changes
* ----- ---- -------- -------------------------------------------------------
* 1.00  djs 2022/07/16 Initial release
*
*
* </pre>
*
* @note
*
******************************************************************************/

#ifndef __COMMON_H__
#define __COMMON_H__

/***************************** Include Files *********************************/
#include "stm32f407xx.h"


/************************** Constant Definitions *****************************/
#define     FALSE   (0!=0)
#define     TRUE    (0 == 0)

/**************************** Type Definitions *******************************/
typedef uint32_t    uint32;
typedef uint16_t    uint16;
typedef uint8_t     uint8;
typedef char        boolean;



/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/
void DelayWait(uint32 cnt);


/************************** Function Inline **********************************/


/************************** Variable Definitions *****************************/


#endif

