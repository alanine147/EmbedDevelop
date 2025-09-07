/************************************************************************
 * @file    cdd_log.h
 * @author  gcg
 * @brief   api of output log
 * @version 1.0.0
 * @date    2025-09-06
 ************************************************************************/

#ifndef CDD_LOG_H
#define CDD_LOG_H

/************************************************************************
 * Includes
 ************************************************************************/
// System includes

// Project includes
#include "common.h"
/************************************************************************
 * Macros & Defines
 ************************************************************************/
#define CDD_LOG_MODULE_NUMBER_MAX   (40U)

#define sys_log(...)   CDD_LOG_String(__VA_ARGS__)
/************************************************************************
 * Typedefs & Enums
 ************************************************************************/
typedef struct CDD_LOG_CfgTag
{
    uint8 rbUart;               /* uart ring buffer id */
    uint32 bufferSize;          /* buffer size */
    uint8 *buffer;              /* buffer */
} CDD_LOG_Cfg;

typedef struct CDD_LOG_Tag
{
    boolean isInit;
    uint32 bufferUsed;
    boolean readFromNvm;
} CDD_LOG;
/************************************************************************
 * Global Variable Declarations
 ************************************************************************/

/************************************************************************
 * Function Prototypes
 ************************************************************************/
void CDD_LOG_String(const char *fmt, ...);
void CDD_LOG_Printf(const char *fmt, ...);
int CDD_LOG_Printf_Ext(const char *fmt, ...);
void CDD_LOG_MainFunction(void);
/************************************************************************
 * Inline function declarations
 ************************************************************************/

#endif // CDD_LOG_H
