/*
 * FreeModbus Libary: STM32F407 BARE Port
 * Based on the FreeModbus BARE demo port.
 */

/* ----------------------- Platform includes --------------------------------*/
#include "communication/modbus/port.h"
#include "tim.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
static USHORT usTimerTimeout50us;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTimeOut50us )
{
    if( usTimeOut50us == 0U )
    {
        return FALSE;
    }

    usTimerTimeout50us = usTimeOut50us;
    return TRUE;
}

void
vMBPortTimersEnable( void )
{
    ( void )HAL_TIM_Base_Stop_IT( &htim7 );
    __HAL_TIM_SET_AUTORELOAD( &htim7, ( uint32_t )usTimerTimeout50us - 1UL );
    __HAL_TIM_SET_COUNTER( &htim7, 0UL );
    __HAL_TIM_CLEAR_FLAG( &htim7, TIM_FLAG_UPDATE );
    ( void )HAL_TIM_Base_Start_IT( &htim7 );
}

void
vMBPortTimersDisable( void )
{
    ( void )HAL_TIM_Base_Stop_IT( &htim7 );
    __HAL_TIM_SET_COUNTER( &htim7, 0UL );
    __HAL_TIM_CLEAR_FLAG( &htim7, TIM_FLAG_UPDATE );
}

void
xMBPortTimersClose( void )
{
    vMBPortTimersDisable(  );
}

void
vMBPortTimersDelay( USHORT usTimeOutMS )
{
    HAL_Delay( usTimeOutMS );
}

void
HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef * htim )
{
    if( htim->Instance == TIM7 )
    {
        ( void )pxMBPortCBTimerExpired(  );
    }
}
