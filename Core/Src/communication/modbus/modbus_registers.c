/*
 * FreeModbus input register mapping for the temperature measurement module.
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdint.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "communication/modbus/modbus_registers.h"

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
    USHORT usChannelIndex;
    USHORT usTemperatureIndex;
    USHORT usResistanceIndex;
    USHORT usStatusIndex;
    xMBFloatValue xResistance;
    xMBFloatValue xTemperature;

    if( ( ucADCIndex >= MB_ADC_COUNT ) || ( ucChannel >= MB_CHANNEL_COUNT ) )
    {
        return;
    }

    usChannelIndex = ( USHORT )( ucADCIndex * MB_CHANNEL_COUNT + ucChannel );
    usTemperatureIndex = ( USHORT )( MB_INPUT_TEMPERATURE_OFFSET
                                     + usChannelIndex * 2U );
    usResistanceIndex = ( USHORT )( MB_INPUT_RESISTANCE_OFFSET
                                    + usChannelIndex * 2U );
    usStatusIndex = ( USHORT )( MB_INPUT_STATUS_OFFSET + usChannelIndex );
    xResistance.fValue = fResistance;
    xTemperature.fValue = fTemperature;

    usRegInputBuf[usTemperatureIndex] =
        ( USHORT )( xTemperature.ulValue >> 16 );
    usRegInputBuf[usTemperatureIndex + 1U] =
        ( USHORT )( xTemperature.ulValue & 0xFFFFU );
    usRegInputBuf[usResistanceIndex] =
        ( USHORT )( xResistance.ulValue >> 16 );
    usRegInputBuf[usResistanceIndex + 1U] =
        ( USHORT )( xResistance.ulValue & 0xFFFFU );
    usRegInputBuf[usStatusIndex] = MB_CHANNEL_STATUS_VALID;
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
