#include "common/system_config.h"

#include "drivers/ad4130.h"
#include "application/sensor_adc.h"
#include "application/sensor_read.h"


typedef struct
{
	SystemCommunicationMode_t communication_mode;
	uint8_t modbus_rtu_slave_address;
	uint8_t modbus_rtu_port;
	uint32_t modbus_rtu_baud_rate;
	ModbusRTUParity_t modbus_rtu_parity;
	uint8_t measurement_log_enabled;
	SystemMeasurementMode_t measurement_mode;
} SystemConfig_t;

static SystemConfig_t system_config = {
	.communication_mode = SYSTEM_COMMUNICATION_MODE_MODBUS_RTU,
	.modbus_rtu_slave_address = 1U,
	.modbus_rtu_port = 0U,
	.modbus_rtu_baud_rate = 115200UL,
	.modbus_rtu_parity = MODBUS_RTU_PARITY_EVEN,
	.measurement_log_enabled = 0U,
	.measurement_mode = SYSTEM_MEASUREMENT_MODE_NORMAL
};

static ErrorCode_t set_measurement_mode(
		SystemMeasurementMode_t mode,
		uint8_t force
);


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

uint8_t system_config_get_measurement_log_enabled(void)
{
	return system_config.measurement_log_enabled;
}

void system_config_set_measurement_log_enabled(uint8_t enabled)
{
	system_config.measurement_log_enabled = (enabled != 0U) ? 1U : 0U;
}

SystemMeasurementMode_t system_config_get_measurement_mode(void)
{
	return system_config.measurement_mode;
}

ErrorCode_t system_config_set_measurement_mode(SystemMeasurementMode_t mode)
{
	return set_measurement_mode(mode, 0U);
}

ErrorCode_t system_config_apply_current_measurement_mode(void)
{
	return set_measurement_mode(system_config.measurement_mode, 1U);
}

static ErrorCode_t set_measurement_mode(
		SystemMeasurementMode_t mode,
		uint8_t force
)
{
	ErrorCode_t result;
	uint8_t previous_conversion_mode;
	uint8_t previous_filter;
	uint8_t previous_fifo_watermark;
	uint8_t new_conversion_mode;
	uint8_t new_filter;
	uint8_t new_fifo_watermark;

	switch (mode)
	{
		case SYSTEM_MEASUREMENT_MODE_NORMAL:
			new_conversion_mode = AD4130_CONVERSION_MODE_NORMAL;
			new_filter = AD4130_FILTER_POST_FILTER_3;
			new_fifo_watermark = 1U;
			break;

		case SYSTEM_MEASUREMENT_MODE_STANDARD:
			new_conversion_mode = AD4130_CONVERSION_MODE_STANDARD;
			new_filter = AD4130_FILTER_POST_FILTER_3;
			new_fifo_watermark = 1U;
			break;

		case SYSTEM_MEASUREMENT_MODE_FAST:
			new_conversion_mode = AD4130_CONVERSION_MODE_FAST;
			new_filter = AD4130_FILTER_SINC3_REJ60;
			new_fifo_watermark = 4U;
			break;

		default:
			return ERROR_CODE_SYSTEM_CONFIG_UNSUPPORTED_MODE;
	}

	if ((force == 0U) && (mode == system_config.measurement_mode))
	{
		return ERROR_CODE_NONE;
	}

	/* sensor_adc: check initialization state */
	if ((force == 0U) && (sensor_adc_is_initialized() == 0U))
	{
		system_config.measurement_mode = mode;
		/* sensor_read: reset synchronized conversion state */
		sensor_read_reset_synchronized_conversion();
		return ERROR_CODE_NONE;
	}

	switch (system_config.measurement_mode)
	{
		case SYSTEM_MEASUREMENT_MODE_NORMAL:
			previous_conversion_mode = AD4130_CONVERSION_MODE_NORMAL;
			previous_filter = AD4130_FILTER_POST_FILTER_3;
			previous_fifo_watermark = 1U;
			break;

		case SYSTEM_MEASUREMENT_MODE_STANDARD:
			previous_conversion_mode = AD4130_CONVERSION_MODE_STANDARD;
			previous_filter = AD4130_FILTER_POST_FILTER_3;
			previous_fifo_watermark = 1U;
			break;

		case SYSTEM_MEASUREMENT_MODE_FAST:
			previous_conversion_mode = AD4130_CONVERSION_MODE_FAST;
			previous_filter = AD4130_FILTER_SINC3_REJ60;
			previous_fifo_watermark = 4U;
			break;

		default:
			return ERROR_CODE_SYSTEM_CONFIG_UNSUPPORTED_MODE;
	}

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		/* ad4130: disable FIFO */
		result = AD4130_FIFO_Disable(adc_device_id);
		if (result == ERROR_CODE_NONE)
		{
			/* ad4130: set conversion mode */
			result = AD4130_Set_Conversion_Mode(
					adc_device_id,
					new_conversion_mode
			);
		}
		if (result == ERROR_CODE_NONE)
		{
			/* ad4130: reset measurement channels */
			result = AD4130_Reset_Measurement_Channels(adc_device_id);
		}
		if (result == ERROR_CODE_NONE)
		{
			/* ad4130: set digital filter */
			result = AD4130_Set_Filter(adc_device_id, new_filter);
		}
		if (result == ERROR_CODE_NONE)
		{
			/* ad4130: enable FIFO */
			result = AD4130_FIFO_Enable(
					adc_device_id,
					new_fifo_watermark
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
				/* ad4130: disable FIFO */
				(void) AD4130_FIFO_Disable(rollback_device_id);
				/* ad4130: restore conversion mode */
				(void) AD4130_Set_Conversion_Mode(
						rollback_device_id,
						previous_conversion_mode
				);
				/* ad4130: restore digital filter */
				(void) AD4130_Set_Filter(
						rollback_device_id,
						previous_filter
				);
				/* ad4130: reset measurement channels */
				(void) AD4130_Reset_Measurement_Channels(
						rollback_device_id
				);
				/* ad4130: enable FIFO */
				(void) AD4130_FIFO_Enable(
						rollback_device_id,
						previous_fifo_watermark
				);
			}
			/* sensor_read: reset synchronized conversion state */
			sensor_read_reset_synchronized_conversion();

			return result;
		}
	}

	system_config.measurement_mode = mode;
	/* sensor_read: reset synchronized conversion state */
	sensor_read_reset_synchronized_conversion();

	return ERROR_CODE_NONE;
}
