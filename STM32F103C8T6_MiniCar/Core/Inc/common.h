/************************************************************************
 * @file    common.h
 * @author  gcg
 * @brief   common defines.
 * @version 1.0.0
 * @date    2025-09-06
 ************************************************************************/

#ifndef COMMON_H
#define COMMON_H

/************************************************************************
 * Includes
 ************************************************************************/
// System includes
#include "stm32f103xb.h"
#include <stdlib.h>
// Project includes

/************************************************************************
 * Macros & Defines
 ************************************************************************/
#define FALSE           (0!=0)
#define TRUE            (0 == 0)
#define NULL_PTR        (NULL)
#define E_OK            (0u)
#define E_NOT_OK        (1u)
#define E_NULL_POINTER  (2u)
#define E_STATUS        (3u)
#define E_CONFIG        (4u)
#define E_PARAMETER     (5u)
#define E_FULL          (6u)
#define E_EMPTY         (7u)
/************************************************************************
 * Typedefs & Enums
 ************************************************************************/
typedef unsigned char boolean;

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;
typedef unsigned long long uint64;
typedef signed char sint8;
typedef signed short sint16;
typedef signed long sint32;
typedef signed long long sint64;
typedef unsigned long uint8_least;
typedef unsigned long uint16_least;
typedef unsigned long uint32_least;
typedef signed long sint8_least;
typedef signed long sint16_least;
typedef signed long sint32_least;

typedef float float32;
typedef double float64;

typedef uint8 Std_ReturnType;

/************************************************************************
 * Global Variable Declarations
 ************************************************************************/

/************************************************************************
 * Function Prototypes
 ************************************************************************/

/************************************************************************
 * Inline function declarations
 ************************************************************************/

#endif // COMMON_H
