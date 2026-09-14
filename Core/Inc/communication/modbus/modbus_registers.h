/*
 * FreeModbus input register mapping for the temperature measurement module.
 */

#ifndef _MODBUS_REGISTERS_H
#define _MODBUS_REGISTERS_H

#include <stdint.h>

#define MB_INPUT_REGISTER_START            1U
#define MB_INPUT_REGISTERS_PER_CHANNEL      5U
#define MB_INPUT_REGISTER_COUNT            40U

void vMBRegInputUpdate( uint8_t ucADCIndex, uint8_t ucChannel,
                        float fResistance, float fTemperature );

#endif
