#include "common/system_config.h"

#include "drivers/ad4130.h"
#include "application/sensor_read.h"


typedef struct
{
	SystemCommunicationMode_t communication_mode;
	uint8_t modbus_rtu_slave_address;
	uint8_t modbus_rtu_port;
	uint32_t modbus_rtu_baud_rate;
	ModbusRTUParity_t modbus_rtu_parity;
	uint8_t measurement_log_enabled;
	uint8_t synchronized_measurement_enabled;
} SystemConfig_t;

static SystemConfig_t system_config = {
	.communication_mode = SYSTEM_COMMUNICATION_MODE_MODBUS_RTU,
	.modbus_rtu_slave_address = 1U,
	.modbus_rtu_port = 0U,
	.modbus_rtu_baud_rate = 115200UL,
	.modbus_rtu_parity = MODBUS_RTU_PARITY_EVEN,
	.measurement_log_enabled = 0U,
	.synchronized_measurement_enabled = 0U
};


SystemCommunicationMode_t system_config_get_communication_mode(void)
{
	return system_config.communication_mode;
}

void system_config_set_communication_mode(SystemCommunicationMode_t mode)
{
	system_config.communication_mode = mode;
}

uint8_t system_config_get_modbus_rtu_slave_address(void)
{
	return system_config.modbus_rtu_slave_address;
}

uint8_t system_config_get_modbus_rtu_port(void)
{
	return system_config.modbus_rtu_port;
}

uint32_t system_config_get_modbus_rtu_baud_rate(void)
{
	return system_config.modbus_rtu_baud_rate;
}

ModbusRTUParity_t system_config_get_modbus_rtu_parity(void)
{
	return system_config.modbus_rtu_parity;
}

uint8_t system_config_measurement_log_enabled(void)
{
	return system_config.measurement_log_enabled;
}

void system_config_set_measurement_log_enabled(uint8_t enabled)
{
	system_config.measurement_log_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t system_config_synchronized_measurement_enabled(void)
{
	return system_config.synchronized_measurement_enabled;
}

ErrorCode_t system_config_set_synchronized_measurement_enabled(uint8_t enabled)
{
	ErrorCode_t result;
	uint8_t previous_mode;
	uint8_t new_mode;

	enabled = (enabled != 0U) ? 1U : 0U;
	if (enabled == system_config.synchronized_measurement_enabled)
	{
		return ERROR_CODE_NONE;
	}

	if (system_config.synchronized_measurement_enabled != 0U)
	{
		previous_mode = AD4130_CONVERSION_MODE_SYNC;
	}
	else
	{
		previous_mode = AD4130_CONVERSION_MODE_NORMAL;
	}

	if (enabled != 0U)
	{
		new_mode = AD4130_CONVERSION_MODE_SYNC;
	}
	else
	{
		new_mode = AD4130_CONVERSION_MODE_NORMAL;
	}

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		result = AD4130_FIFO_Disable(adc_device_id);
		if (result == ERROR_CODE_NONE)
		{
			result = AD4130_Set_Conversion_Mode(adc_device_id, new_mode);
		}
		if (result == ERROR_CODE_NONE)
		{
			result = AD4130_FIFO_Enable(
					adc_device_id,
					AD4130_SENSOR_CHANNEL_COUNT
			);
		}
		if (result != ERROR_CODE_NONE)
		{
			for (
					uint8_t rollback_device_id = AD4130_DEVICE_ID_MIN;
					rollback_device_id <= AD4130_DEVICE_ID_MAX;
					rollback_device_id++
			)
			{
				(void) AD4130_FIFO_Disable(rollback_device_id);
				(void) AD4130_Set_Conversion_Mode(
						rollback_device_id,
						previous_mode
				);
				(void) AD4130_FIFO_Enable(
						rollback_device_id,
						AD4130_SENSOR_CHANNEL_COUNT
				);
			}

			return result;
		}
	}

	system_config.synchronized_measurement_enabled = enabled;
	sensor_read_reset_synchronized_conversion();

	return ERROR_CODE_NONE;
}
