#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	SYSTEM_COMMUNICATION_MODE_MODBUS_RTU = 0,
	SYSTEM_COMMUNICATION_MODE_MODBUS_TCP = 1,
	SYSTEM_COMMUNICATION_MODE_PROFIBUS = 2
} SystemCommunicationMode_t;

typedef enum
{
	MODBUS_RTU_PARITY_NONE = 0,
	MODBUS_RTU_PARITY_ODD = 1,
	MODBUS_RTU_PARITY_EVEN = 2
} ModbusRTUParity_t;


SystemCommunicationMode_t system_config_get_communication_mode(void);

void system_config_set_communication_mode(SystemCommunicationMode_t mode);

uint8_t system_config_get_modbus_rtu_slave_address(void);

uint8_t system_config_get_modbus_rtu_port(void);

uint32_t system_config_get_modbus_rtu_baud_rate(void);

ModbusRTUParity_t system_config_get_modbus_rtu_parity(void);

uint8_t system_config_measurement_log_enabled(void);

void system_config_set_measurement_log_enabled(uint8_t enabled);

uint8_t system_config_synchronized_measurement_enabled(void);

void system_config_set_synchronized_measurement_enabled(uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif  /* SYSTEM_CONFIG_H */
