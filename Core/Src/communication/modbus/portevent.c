/*
 * FreeModbus Libary: STM32F407 BARE Port
 * Based on the FreeModbus BARE demo port.
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
static volatile eMBEventType eQueuedEvent;
static volatile BOOL xEventInQueue;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortEventInit( void )
{
    xEventInQueue = FALSE;
    return TRUE;
}

BOOL
xMBPortEventPost( eMBEventType eEvent )
{
    eQueuedEvent = eEvent;
    xEventInQueue = TRUE;
    return TRUE;
}

BOOL
xMBPortEventGet( eMBEventType * eEvent )
{
    BOOL xEventHappened = FALSE;

    ENTER_CRITICAL_SECTION(  );
    if( xEventInQueue )
    {
        *eEvent = eQueuedEvent;
        xEventInQueue = FALSE;
        xEventHappened = TRUE;
    }
    EXIT_CRITICAL_SECTION(  );

    return xEventHappened;
}
