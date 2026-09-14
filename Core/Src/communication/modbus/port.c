/*
 * FreeModbus Libary: STM32F407 BARE Port
 * Based on the FreeModbus BARE demo port.
 */

#include "port.h"

static ULONG ulCriticalNesting;
static ULONG ulSavedPrimask;

void
vMBPortEnterCritical( void )
{
    ULONG ulPrimask;

    ulPrimask = __get_PRIMASK(  );
    __disable_irq(  );

    if( ulCriticalNesting == 0UL )
    {
        ulSavedPrimask = ulPrimask;
    }
    ulCriticalNesting++;
}

void
vMBPortExitCritical( void )
{
    if( ulCriticalNesting > 0UL )
    {
        ulCriticalNesting--;
        if( ( ulCriticalNesting == 0UL ) && ( ulSavedPrimask == 0UL ) )
        {
            __enable_irq(  );
        }
    }
}
