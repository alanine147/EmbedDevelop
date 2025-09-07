/************************************************************************
 * @file    cdd_rb.h
 * @author  gcg
 * @brief   api definations of ring buffer
 * @version 1.0.0
 * @date    2025-09-06
 ************************************************************************/

#ifndef CDD_RB_H
#define CDD_RB_H

/************************************************************************
 * Includes
 ************************************************************************/
// System includes

// Project includes
#include "common.h"
/************************************************************************
 * Macros & Defines
 ************************************************************************/

/************************************************************************
 * Typedefs & Enums
 ************************************************************************/
typedef enum CDD_RB_Buffer_EnumTag
{
    CDD_RB_BUFFER_LOGGER_UART,
    CDD_RB_BUFFER_NUMBER
} CDD_RB_Buffer_Enum;
typedef struct CDD_RB_BufferTag
{
    uint32 headIndex;                   /* head location of buffer */
    uint32 tailIndex;                   /* tail location of buffer */
} CDD_RB_Buffer;

/* configuration of ring buffer */
typedef struct CDD_RB_BufferCfgTag
{
    uint8 *buffer;                      /* buffer location to store element in queue */
    uint8 *bufferOverflow;              /* buffer location overflow */
    uint32 elementLength;               /* element length in byte */
    uint32 elementNumber;               /* how many element can have in ring buffer */
    boolean popMethod;                  /* when pop operation, as many as possible is popped */
    boolean pushMethod;                 /* when push operation, as many as possible is pushed */
} CDD_RB_BufferCfg;

/* runtime data of module */
typedef struct CDD_RB_Tag
{
    boolean isInit;                     /* whether module is initialized */
} CDD_RB;

/* configuration of module */
typedef struct CDD_RB_CfgTag
{
    uint8 bufferNumber;                 /* ring buffer number */
    const CDD_RB_BufferCfg *bufferCfgs; /* ring buffer configuartion array */
    CDD_RB_Buffer *buffers;             /* ring buffer data array */
} CDD_RB_Cfg;
/************************************************************************
 * Global Variable Declarations
 ************************************************************************/
extern const CDD_RB_Cfg cdd_rb_cfg;
/************************************************************************
 * Function Prototypes
 ************************************************************************/
Std_ReturnType CDD_RB_Init(const CDD_RB_Cfg *cfg);
Std_ReturnType CDD_RB_Push(uint8 id, uint8 *data, uint32 elementNumber);
Std_ReturnType CDD_RB_Peek(uint8 id, uint8 *data, uint32 elementNumber);
Std_ReturnType CDD_RB_Pop(uint8 id, uint8 *data, uint32 elementNumber);
Std_ReturnType CDD_RB_NumberGet(uint8 id, uint32 *number);
Std_ReturnType CDD_RB_Clear(uint8 id);
/************************************************************************
 * Inline function declarations
 ************************************************************************/

#endif // CDD_RB_H
