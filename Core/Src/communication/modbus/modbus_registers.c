/*
 * FreeModbus input register mapping for the temperature measurement module.
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdint.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "communication/modbus/modbus_registers.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_CHANNEL_STATUS_VALID             1U
#define MB_CHANNEL_STATUS_ERROR             0xFFFFU  /* -1 as int16_t */

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

    if( ( ucADCIndex >= AD4130_DEVICE_COUNT )
        || ( ucChannel >= AD4130_SENSOR_CHANNEL_COUNT ) )
    {
        return;
    }

    usChannelIndex = ( USHORT )( ucADCIndex
                                 * AD4130_SENSOR_CHANNEL_COUNT + ucChannel );
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

void
vMBRegInputSetChannelError( uint8_t ucADCIndex, uint8_t ucChannel )
{
    USHORT usChannelIndex;
    USHORT usStatusIndex;

    if( ( ucADCIndex >= AD4130_DEVICE_COUNT )
        || ( ucChannel >= AD4130_SENSOR_CHANNEL_COUNT ) )
    {
        return;
    }

    usChannelIndex = ( USHORT )( ucADCIndex
                                 * AD4130_SENSOR_CHANNEL_COUNT + ucChannel );
    usStatusIndex = ( USHORT )( MB_INPUT_STATUS_OFFSET + usChannelIndex );
    usRegInputBuf[usStatusIndex] = MB_CHANNEL_STATUS_ERROR;
}

void
vMBRegInputSetADCError( uint8_t ucADCIndex )
{
    uint8_t ucChannel;

    if( ucADCIndex >= AD4130_DEVICE_COUNT )
    {
        return;
    }

    for( ucChannel = 0U; ucChannel < AD4130_SENSOR_CHANNEL_COUNT; ucChannel++ )
    {
        vMBRegInputSetChannelError( ucADCIndex, ucChannel );
    }
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
