/*
 * FreeModbus input register mapping for the temperature measurement module.
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdint.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "modbus_registers.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_ADC_COUNT                        2U
#define MB_CHANNEL_COUNT                    4U
#define MB_CHANNEL_STATUS_VALID             1U

/* ----------------------- Type definitions ---------------------------------*/
typedef union
{
    float fValue;
    uint32_t ulValue;
} xMBFloatValue;

/* ----------------------- Static variables ---------------------------------*/
static USHORT usRegInputBuf[MB_INPUT_REGISTER_COUNT];

/* ----------------------- Start implementation -----------------------------*/
void
vMBRegInputUpdate( uint8_t ucADCIndex, uint8_t ucChannel,
                   float fResistance, float fTemperature )
{
    USHORT usRegIndex;
    xMBFloatValue xResistance;
    xMBFloatValue xTemperature;

    if( ( ucADCIndex >= MB_ADC_COUNT ) || ( ucChannel >= MB_CHANNEL_COUNT ) )
    {
        return;
    }

    usRegIndex = ( USHORT )( ( ucADCIndex * MB_CHANNEL_COUNT + ucChannel )
                             * MB_INPUT_REGISTERS_PER_CHANNEL );
    xResistance.fValue = fResistance;
    xTemperature.fValue = fTemperature;

    usRegInputBuf[usRegIndex] = MB_CHANNEL_STATUS_VALID;
    usRegInputBuf[usRegIndex + 1U] = ( USHORT )( xResistance.ulValue >> 16 );
    usRegInputBuf[usRegIndex + 2U] = ( USHORT )( xResistance.ulValue & 0xFFFFU );
    usRegInputBuf[usRegIndex + 3U] = ( USHORT )( xTemperature.ulValue >> 16 );
    usRegInputBuf[usRegIndex + 4U] = ( USHORT )( xTemperature.ulValue & 0xFFFFU );
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT usRegIndex;

    if( ( usAddress >= MB_INPUT_REGISTER_START )
        && ( usAddress + usNRegs
             <= MB_INPUT_REGISTER_START + MB_INPUT_REGISTER_COUNT ) )
    {
        usRegIndex = ( USHORT )( usAddress - MB_INPUT_REGISTER_START );
        while( usNRegs > 0U )
        {
            *pucRegBuffer++ = ( UCHAR )( usRegInputBuf[usRegIndex] >> 8 );
            *pucRegBuffer++ = ( UCHAR )( usRegInputBuf[usRegIndex] & 0xFFU );
            usRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}
