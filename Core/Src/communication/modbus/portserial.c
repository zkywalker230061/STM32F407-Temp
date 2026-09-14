/*
 * FreeModbus Libary: STM32F407 BARE Port
 * Based on the FreeModbus BARE demo port.
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "usart.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    __HAL_UART_DISABLE_IT( &huart2, UART_IT_RXNE );
    __HAL_UART_DISABLE_IT( &huart2, UART_IT_TXE );

    if( xRxEnable )
    {
        __HAL_UART_ENABLE_IT( &huart2, UART_IT_RXNE );
    }
    if( xTxEnable )
    {
        __HAL_UART_ENABLE_IT( &huart2, UART_IT_TXE );
    }
}

BOOL
xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits,
                   eMBParity eParity )
{
    if( ( ucPort != 0U ) || ( ucDataBits != 8U ) )
    {
        return FALSE;
    }

    huart2.Init.BaudRate = ulBaudRate;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    switch( eParity )
    {
    case MB_PAR_NONE:
        huart2.Init.WordLength = UART_WORDLENGTH_8B;
        huart2.Init.StopBits = UART_STOPBITS_2;
        huart2.Init.Parity = UART_PARITY_NONE;
        break;

    case MB_PAR_ODD:
        huart2.Init.WordLength = UART_WORDLENGTH_9B;
        huart2.Init.StopBits = UART_STOPBITS_1;
        huart2.Init.Parity = UART_PARITY_ODD;
        break;

    case MB_PAR_EVEN:
        huart2.Init.WordLength = UART_WORDLENGTH_9B;
        huart2.Init.StopBits = UART_STOPBITS_1;
        huart2.Init.Parity = UART_PARITY_EVEN;
        break;

    default:
        return FALSE;
    }

    if( HAL_UART_Init( &huart2 ) != HAL_OK )
    {
        return FALSE;
    }

    vMBPortSerialEnable( FALSE, FALSE );
    return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    huart2.Instance->DR = ( uint16_t )( ( UCHAR )ucByte );
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    if( pucByte == NULL )
    {
        return FALSE;
    }

    *pucByte = ( CHAR )( huart2.Instance->DR & 0xFFU );
    return TRUE;
}

void
xMBPortSerialClose( void )
{
    vMBPortSerialEnable( FALSE, FALSE );
}

void
vMBPortClose( void )
{
    xMBPortSerialClose(  );
    xMBPortTimersClose(  );
}

void
vMBPortSerialIRQHandler( void )
{
    ULONG ulStatus;
    ULONG ulControl;

    ulStatus = huart2.Instance->SR;
    ulControl = huart2.Instance->CR1;

    if( ( ( ulStatus & USART_SR_RXNE ) != 0UL )
        && ( ( ulControl & USART_CR1_RXNEIE ) != 0UL ) )
    {
        ( void )pxMBFrameCBByteReceived(  );
    }

    if( ( ( ulStatus & USART_SR_TXE ) != 0UL )
        && ( ( ulControl & USART_CR1_TXEIE ) != 0UL ) )
    {
        ( void )pxMBFrameCBTransmitterEmpty(  );
    }

    if( ( ( ulStatus & ( USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE ) ) != 0UL )
        && ( ( ulStatus & USART_SR_RXNE ) == 0UL ) )
    {
        volatile ULONG ulData;

        ulData = huart2.Instance->DR;
        ( void )ulData;
    }
}
