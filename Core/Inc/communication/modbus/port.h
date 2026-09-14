/*
 * FreeModbus Libary: STM32F407 BARE Port
 * Based on the FreeModbus BARE demo port.
 */

#ifndef _PORT_H
#define _PORT_H

#include <assert.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define INLINE                      inline

#ifdef __cplusplus
#define PR_BEGIN_EXTERN_C           extern "C" {
#define PR_END_EXTERN_C             }
#else
#define PR_BEGIN_EXTERN_C
#define PR_END_EXTERN_C
#endif

#define ENTER_CRITICAL_SECTION( )   vMBPortEnterCritical( )
#define EXIT_CRITICAL_SECTION( )    vMBPortExitCritical( )

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

void vMBPortEnterCritical( void );
void vMBPortExitCritical( void );
void vMBPortSerialIRQHandler( void );

#endif
