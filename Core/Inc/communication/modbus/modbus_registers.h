/*
 * FreeModbus input register mapping for the temperature measurement module.
 */

#ifndef _MODBUS_REGISTERS_H
#define _MODBUS_REGISTERS_H

#include <stdint.h>

#include "common/ad4130_config_file.h"

#define MB_INPUT_REGISTER_START             1U
#define MB_INPUT_TEMPERATURE_OFFSET          0U
#define MB_INPUT_RESISTANCE_OFFSET          ( AD4130_TOTAL_SENSOR_COUNT * 2U )
#define MB_INPUT_STATUS_OFFSET              ( AD4130_TOTAL_SENSOR_COUNT * 4U )
#define MB_INPUT_REGISTER_COUNT             ( AD4130_TOTAL_SENSOR_COUNT * 5U )

void vMBRegInputUpdate( uint8_t ucADCIndex, uint8_t ucChannel,
                        float fResistance, float fTemperature );
void vMBRegInputSetChannelError( uint8_t ucADCIndex, uint8_t ucChannel );
void vMBRegInputSetADCError( uint8_t ucADCIndex );

#endif
