#include "application/sensor_read.h"

#include <stdio.h>

#include "common/ad4130_config_file.h"
#include "common/system_config.h"
#include "drivers/ad4130.h"
#include "drivers/ad4130_measurement.h"
#include "application/sensor_coeffs.h"
#include "application/sensor_fit.h"
#include "communication/modbus/modbus_registers.h"


static float resistance[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT];
static float temperature[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT];
static float forward_resistance
		[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT];
static uint8_t forward_resistance_valid
		[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT];
static uint8_t synchronized_conversion_active = 0U;
static uint8_t synchronized_channel = AD4130_CHANNEL_MIN;
static uint8_t stad_standard_channel = AD4130_CHANNEL_MIN;
static uint8_t stad_measurement_channel = AD4130_CHANNEL_MIN;

static ErrorCode_t read_sensor_norm(void);
static ErrorCode_t read_sensor_stad(void);
static ErrorCode_t read_sensor_fast(void);
static ErrorCode_t get_sensor_channel(
		uint8_t channel,
		uint8_t *sensor_channel
);
static ErrorCode_t switch_next_channel(void);
static ErrorCode_t switch_next_channel_stad(void);
static ErrorCode_t recover_fifo(
		uint8_t error_adc_device_id,
		ErrorCode_t error,
		uint8_t synchronized
);


ErrorCode_t read_sensor(void)
{
	/* system_config: get measurement mode */
	switch (system_config_get_measurement_mode())
	{
		case SYSTEM_MEASUREMENT_MODE_NORMAL:
			return read_sensor_norm();

		case SYSTEM_MEASUREMENT_MODE_STANDARD:
			return read_sensor_stad();

		case SYSTEM_MEASUREMENT_MODE_FAST:
			return read_sensor_fast();

		default:
			return ERROR_CODE_SYSTEM_CONFIG_UNSUPPORTED_MODE;
	}
}

static ErrorCode_t read_sensor_norm(void)
{
	ErrorCode_t result;
	ErrorCode_t recovery_result;
	ErrorCode_t fit_result;
	ErrorCode_t sample_results[AD4130_SENSOR_CHANNEL_COUNT];
	uint8_t channels[AD4130_SENSOR_CHANNEL_COUNT];
	float resistances[AD4130_SENSOR_CHANNEL_COUNT];
	uint8_t fifo_ready;
	uint8_t channel;
	uint8_t sensor_channel;
	float measured_resistance;
	float measured_temperature;
	uint8_t read_count = 0U;

	if (synchronized_conversion_active == 0U)
	{
		/* ad4130: start synchronized sequence */
		AD4130_Synchronize();
		synchronized_conversion_active = 1U;

		return ERROR_CODE_MEASUREMENT_NOT_READY;
	}

	/* ad4130: check FIFO ready */
	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		result = AD4130_FIFO_Ready(
				adc_device_id,
				&fifo_ready
		);
		if (result != ERROR_CODE_NONE)
		{
			recovery_result = recover_fifo(
					adc_device_id,
					result,
					1U
			);
			if (recovery_result != ERROR_CODE_NONE)
			{
				return recovery_result;
			}

			if (
					(result == ERROR_CODE_AD4130_FIFO_WRITE)
					|| (result == ERROR_CODE_AD4130_FIFO_READ)
					|| (result == ERROR_CODE_AD4130_FIFO_OVERRUN)
			)
			{
				return ERROR_CODE_MEASUREMENT_NOT_READY;
			}

			return result;
		}
		if (fifo_ready == 0U)
		{
			return ERROR_CODE_MEASUREMENT_NOT_READY;
		}
	}

	synchronized_conversion_active = 0U;

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		/* ad4130_measurement: read resistance from FIFO */
		result = AD4130_Read_Resistance_FIFO(
				adc_device_id,
				channels,
				resistances,
				sample_results,
				1U,
				(synchronized_channel >= AD4130_REVERSE_CHANNEL_MIN)
						? 1U
						: 0U,
				0U
		);
		if (result != ERROR_CODE_NONE)
		{
			recovery_result = recover_fifo(
					adc_device_id,
					result,
					1U
			);
			if (recovery_result != ERROR_CODE_NONE)
			{
				return recovery_result;
			}

			if (
					(result == ERROR_CODE_AD4130_FIFO_WRITE)
					|| (result == ERROR_CODE_AD4130_FIFO_READ)
					|| (result == ERROR_CODE_AD4130_FIFO_OVERRUN)
			)
			{
				return ERROR_CODE_MEASUREMENT_NOT_READY;
			}

			return result;
		}
		if (channels[0] != synchronized_channel)
		{
			result = ERROR_CODE_AD4130_FIFO_READ;
			recovery_result = recover_fifo(
					adc_device_id,
					result,
					1U
			);
			if (recovery_result != ERROR_CODE_NONE)
			{
				return recovery_result;
			}

			return ERROR_CODE_MEASUREMENT_NOT_READY;
		}

		for (uint8_t sample = 0U; sample < 1U; sample++)
		{
			channel = channels[sample];
			/* get the physical sensor channel */
			result = get_sensor_channel(channel, &sensor_channel);
			if (result != ERROR_CODE_NONE)
			{
				return result;
			}
			if (channel < AD4130_REVERSE_CHANNEL_MIN)
			{
				forward_resistance_valid
						[adc_device_id-1U][sensor_channel] = 0U;
			}
			measured_resistance = resistances[sample];
			result = sample_results[sample];
			if (result == ERROR_CODE_MEASUREMENT_NOT_READY)
			{
				continue;
			}
			if (result != ERROR_CODE_NONE)
			{
				if (channel >= AD4130_REVERSE_CHANNEL_MIN)
				{
					forward_resistance_valid
							[adc_device_id-1U][sensor_channel] = 0U;
				}
				if (
						(result == ERROR_CODE_MEASUREMENT_ILLEGAL_RANGE)
						|| (result == ERROR_CODE_MEASUREMENT_BELOW_RANGE)
						|| (result == ERROR_CODE_MEASUREMENT_ABOVE_RANGE)
				)
				{
					printf(
							"%d: ADC %u CHANNEL_%u read error\r\n",
							(int) result,
							(unsigned int) adc_device_id,
							(unsigned int) channel
					);
					/* modbus_registers: set channel error */
					vMBRegInputSetChannelError(
							adc_device_id-1U,
							sensor_channel
					);
				}
				else
				{
					printf(
							"%d: ADC %u read error\r\n",
							(int) result,
							(unsigned int) adc_device_id
					);
					/* modbus_registers: set ADC error */
					vMBRegInputSetADCError(adc_device_id-1U);
				}

				if (
						(result == ERROR_CODE_AD4130_ILLEGAL_PARAM)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_RANGE)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_WRITE_LENGTH)
						|| (result == ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM)
						|| (result == ERROR_CODE_MEASUREMENT_ILLEGAL_RANGE)
				)
				{
					return result;
				}

				continue;
			}

			read_count++;

			if (channel < AD4130_REVERSE_CHANNEL_MIN)
			{
				/* store the forward resistance */
				forward_resistance
						[adc_device_id-1U][sensor_channel] = measured_resistance;
				forward_resistance_valid
						[adc_device_id-1U][sensor_channel] = 1U;
			}
			else if (
					forward_resistance_valid
							[adc_device_id-1U][sensor_channel] != 0U
			)
			{
				/* average the forward and reverse resistances */
				resistance[adc_device_id-1U][sensor_channel] = (
						forward_resistance[adc_device_id-1U][sensor_channel]
						+ measured_resistance
				) / 2.0f;
				forward_resistance_valid
						[adc_device_id-1U][sensor_channel] = 0U;

				/* sensor_coeffs: get sensor curve */
				/* sensor_fit: average temperature fit */
				fit_result = resistance_to_temperature(
						resistance[adc_device_id-1U][sensor_channel],
						sensor_coeffs_get_curve(adc_device_id, sensor_channel),
						&temperature[adc_device_id-1U][sensor_channel]
				);
				if (fit_result != ERROR_CODE_NONE)
				{
					printf(
							"%d: ADC %u CHANNEL_%u fit error\r\n",
							(int) fit_result,
							(unsigned int) adc_device_id,
							(unsigned int) channel
					);
					/* modbus_registers: set channel error */
					vMBRegInputSetChannelError(
							adc_device_id-1U,
							sensor_channel
					);
					if (fit_result == ERROR_CODE_SENSOR_FIT_ILLEGAL_PARAM)
					{
						return fit_result;
					}
				}
				else
				{
					/* modbus_registers: update averaged input registers */
					vMBRegInputUpdate(
							adc_device_id-1U,
							sensor_channel,
							resistance[adc_device_id-1U][sensor_channel],
							temperature[adc_device_id-1U][sensor_channel]
					);
				}
			}

			/* sensor_coeffs: get sensor curve */
			/* sensor_fit: temperature fit */
			fit_result = resistance_to_temperature(
					measured_resistance,
					sensor_coeffs_get_curve(adc_device_id, sensor_channel),
					&measured_temperature
			);
			if (fit_result != ERROR_CODE_NONE)
			{
				printf(
						"%d: ADC %u CHANNEL_%u fit error\r\n",
						(int) fit_result,
						(unsigned int) adc_device_id,
						(unsigned int) channel
				);
				if (fit_result == ERROR_CODE_SENSOR_FIT_ILLEGAL_PARAM)
				{
					return fit_result;
				}

				continue;
			}

			/* system_config: check measurement log state */
			if (system_config_get_measurement_log_enabled() != 0U)
			{
				printf(
						"%u-%u: R-%.4f ohm, T-%.5f K\r\n",
						(unsigned int) adc_device_id,
						(unsigned int) (channel+1U),
						(double) measured_resistance,
						(double) measured_temperature
				);
			}
		}
	}

	/* switch to the next measurement channel */
	result = switch_next_channel();
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	if (read_count == 0U)
	{
		return ERROR_CODE_MEASUREMENT_NOT_READY;
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t read_sensor_stad(void)
{
	ErrorCode_t result;
	ErrorCode_t recovery_result;
	ErrorCode_t fit_result;
	ErrorCode_t sample_results[AD4130_SENSOR_CHANNEL_COUNT];
	uint8_t channels[AD4130_SENSOR_CHANNEL_COUNT];
	float resistances[AD4130_SENSOR_CHANNEL_COUNT];
	uint8_t fifo_ready;
	uint8_t channel;
	uint8_t sensor_channel;
	uint8_t expected_channel;
	float measured_resistance;
	float measured_temperature;
	uint8_t read_count = 0U;

	if (synchronized_conversion_active == 0U)
	{
		/* ad4130: start synchronized sequence */
		AD4130_Synchronize();
		synchronized_conversion_active = 1U;

		return ERROR_CODE_MEASUREMENT_NOT_READY;
	}

	/* ad4130: check FIFO ready */
	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		result = AD4130_FIFO_Ready(
				adc_device_id,
				&fifo_ready
		);
		if (result != ERROR_CODE_NONE)
		{
			recovery_result = recover_fifo(
					adc_device_id,
					result,
					1U
			);
			if (recovery_result != ERROR_CODE_NONE)
			{
				return recovery_result;
			}

			if (
					(result == ERROR_CODE_AD4130_FIFO_WRITE)
					|| (result == ERROR_CODE_AD4130_FIFO_READ)
					|| (result == ERROR_CODE_AD4130_FIFO_OVERRUN)
			)
			{
				return ERROR_CODE_MEASUREMENT_NOT_READY;
			}

			return result;
		}
		if (fifo_ready == 0U)
		{
			return ERROR_CODE_MEASUREMENT_NOT_READY;
		}
	}

	synchronized_conversion_active = 0U;

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		expected_channel = (
				(adc_device_id >= AD4130_STAD_DEVICE_ID_MIN)
				&& (adc_device_id <= AD4130_STAD_DEVICE_ID_MAX)
		) ? stad_standard_channel : stad_measurement_channel;

		/* ad4130_measurement: read resistance from FIFO */
		result = AD4130_Read_Resistance_FIFO(
				adc_device_id,
				channels,
				resistances,
				sample_results,
				1U,
				(expected_channel >= AD4130_REVERSE_CHANNEL_MIN)
						? 1U
						: 0U,
				0U
		);
		if (result != ERROR_CODE_NONE)
		{
			recovery_result = recover_fifo(
					adc_device_id,
					result,
					1U
			);
			if (recovery_result != ERROR_CODE_NONE)
			{
				return recovery_result;
			}

			if (
					(result == ERROR_CODE_AD4130_FIFO_WRITE)
					|| (result == ERROR_CODE_AD4130_FIFO_READ)
					|| (result == ERROR_CODE_AD4130_FIFO_OVERRUN)
			)
			{
				return ERROR_CODE_MEASUREMENT_NOT_READY;
			}

			return result;
		}
		if (channels[0] != expected_channel)
		{
			result = ERROR_CODE_AD4130_FIFO_READ;
			recovery_result = recover_fifo(
					adc_device_id,
					result,
					1U
			);
			if (recovery_result != ERROR_CODE_NONE)
			{
				return recovery_result;
			}

			return ERROR_CODE_MEASUREMENT_NOT_READY;
		}

		for (uint8_t sample = 0U; sample < 1U; sample++)
		{
			channel = channels[sample];
			/* get the physical sensor channel */
			result = get_sensor_channel(channel, &sensor_channel);
			if (result != ERROR_CODE_NONE)
			{
				return result;
			}
			if (channel < AD4130_REVERSE_CHANNEL_MIN)
			{
				forward_resistance_valid
						[adc_device_id-1U][sensor_channel] = 0U;
			}
			measured_resistance = resistances[sample];
			result = sample_results[sample];
			if (result == ERROR_CODE_MEASUREMENT_NOT_READY)
			{
				continue;
			}
			if (result != ERROR_CODE_NONE)
			{
				if (channel >= AD4130_REVERSE_CHANNEL_MIN)
				{
					forward_resistance_valid
							[adc_device_id-1U][sensor_channel] = 0U;
				}
				if (
						(result == ERROR_CODE_MEASUREMENT_ILLEGAL_RANGE)
						|| (result == ERROR_CODE_MEASUREMENT_BELOW_RANGE)
						|| (result == ERROR_CODE_MEASUREMENT_ABOVE_RANGE)
				)
				{
					printf(
							"%d: ADC %u CHANNEL_%u read error\r\n",
							(int) result,
							(unsigned int) adc_device_id,
							(unsigned int) channel
					);
					/* modbus_registers: set channel error */
					vMBRegInputSetChannelError(
							adc_device_id-1U,
							sensor_channel
					);
				}
				else
				{
					printf(
							"%d: ADC %u read error\r\n",
							(int) result,
							(unsigned int) adc_device_id
					);
					/* modbus_registers: set ADC error */
					vMBRegInputSetADCError(adc_device_id-1U);
				}

				if (
						(result == ERROR_CODE_AD4130_ILLEGAL_PARAM)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_RANGE)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_WRITE_LENGTH)
						|| (result == ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM)
						|| (result == ERROR_CODE_MEASUREMENT_ILLEGAL_RANGE)
				)
				{
					return result;
				}

				continue;
			}

			read_count++;

			if (channel < AD4130_REVERSE_CHANNEL_MIN)
			{
				/* store the forward resistance */
				forward_resistance
						[adc_device_id-1U][sensor_channel] = measured_resistance;
				forward_resistance_valid
						[adc_device_id-1U][sensor_channel] = 1U;
			}
			else if (
					forward_resistance_valid
							[adc_device_id-1U][sensor_channel] != 0U
			)
			{
				/* average the forward and reverse resistances */
				resistance[adc_device_id-1U][sensor_channel] = (
						forward_resistance[adc_device_id-1U][sensor_channel]
						+ measured_resistance
				) / 2.0f;
				forward_resistance_valid
						[adc_device_id-1U][sensor_channel] = 0U;

				/* sensor_coeffs: get sensor curve */
				/* sensor_fit: average temperature fit */
				fit_result = resistance_to_temperature(
						resistance[adc_device_id-1U][sensor_channel],
						sensor_coeffs_get_curve(adc_device_id, sensor_channel),
						&temperature[adc_device_id-1U][sensor_channel]
				);
				if (fit_result != ERROR_CODE_NONE)
				{
					printf(
							"%d: ADC %u CHANNEL_%u fit error\r\n",
							(int) fit_result,
							(unsigned int) adc_device_id,
							(unsigned int) channel
					);
					/* modbus_registers: set channel error */
					vMBRegInputSetChannelError(
							adc_device_id-1U,
							sensor_channel
					);
					if (fit_result == ERROR_CODE_SENSOR_FIT_ILLEGAL_PARAM)
					{
						return fit_result;
					}
				}
				else
				{
					/* modbus_registers: update averaged input registers */
					vMBRegInputUpdate(
							adc_device_id-1U,
							sensor_channel,
							resistance[adc_device_id-1U][sensor_channel],
							temperature[adc_device_id-1U][sensor_channel]
					);
				}
			}

			/* sensor_coeffs: get sensor curve */
			/* sensor_fit: temperature fit */
			fit_result = resistance_to_temperature(
					measured_resistance,
					sensor_coeffs_get_curve(adc_device_id, sensor_channel),
					&measured_temperature
			);
			if (fit_result != ERROR_CODE_NONE)
			{
				printf(
						"%d: ADC %u CHANNEL_%u fit error\r\n",
						(int) fit_result,
						(unsigned int) adc_device_id,
						(unsigned int) channel
				);
				if (fit_result == ERROR_CODE_SENSOR_FIT_ILLEGAL_PARAM)
				{
					return fit_result;
				}

				continue;
			}

			/* system_config: check measurement log state */
			if (system_config_get_measurement_log_enabled() != 0U)
			{
				printf(
						"%u-%u: R-%.4f ohm, T-%.5f K\r\n",
						(unsigned int) adc_device_id,
						(unsigned int) (channel+1U),
						(double) measured_resistance,
						(double) measured_temperature
				);
			}
		}
	}

	/* switch to the next measurement channel */
	result = switch_next_channel_stad();
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	if (read_count == 0U)
	{
		return ERROR_CODE_MEASUREMENT_NOT_READY;
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t read_sensor_fast(void)
{
	ErrorCode_t result;
	ErrorCode_t recovery_result;
	ErrorCode_t fit_result;
	ErrorCode_t sample_results[4U];
	uint8_t channels[4U];
	float resistances[4U];
	uint8_t fifo_ready[AD4130_DEVICE_COUNT] = {0U};
	uint8_t channel;
	float measured_resistance;
	float measured_temperature;
	uint8_t read_count = 0U;

	/* ad4130: check FIFO ready */
	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		result = AD4130_FIFO_Ready(
				adc_device_id,
				&fifo_ready[adc_device_id-1U]
		);
		if (result != ERROR_CODE_NONE)
		{
			recovery_result = recover_fifo(
					adc_device_id,
					result,
					0U
			);
			if (recovery_result != ERROR_CODE_NONE)
			{
				return recovery_result;
			}

			if (
					(result == ERROR_CODE_AD4130_FIFO_WRITE)
					|| (result == ERROR_CODE_AD4130_FIFO_READ)
					|| (result == ERROR_CODE_AD4130_FIFO_OVERRUN)
			)
			{
				continue;
			}

			return result;
		}
	}

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		if (fifo_ready[adc_device_id-1U] == 0U)
		{
			continue;
		}

		/* ad4130_measurement: read resistance from FIFO */
		result = AD4130_Read_Resistance_FIFO(
				adc_device_id,
				channels,
				resistances,
				sample_results,
				4U,
				1U,
				1U
		);
		if (result != ERROR_CODE_NONE)
		{
			recovery_result = recover_fifo(
					adc_device_id,
					result,
					0U
			);
			if (recovery_result != ERROR_CODE_NONE)
			{
				return recovery_result;
			}

			if (
					(result == ERROR_CODE_AD4130_FIFO_WRITE)
					|| (result == ERROR_CODE_AD4130_FIFO_READ)
					|| (result == ERROR_CODE_AD4130_FIFO_OVERRUN)
			)
			{
				continue;
			}

			return result;
		}

		for (uint8_t sample = 0U; sample < 4U; sample++)
		{
			channel = channels[sample];
			measured_resistance = resistances[sample];
			result = sample_results[sample];
			if (result == ERROR_CODE_MEASUREMENT_NOT_READY)
			{
				continue;
			}
			if (result != ERROR_CODE_NONE)
			{
				if (
						(result == ERROR_CODE_MEASUREMENT_ILLEGAL_RANGE)
						|| (result == ERROR_CODE_MEASUREMENT_BELOW_RANGE)
						|| (result == ERROR_CODE_MEASUREMENT_ABOVE_RANGE)
				)
				{
					printf(
							"%d: ADC %u CHANNEL_%u read error\r\n",
							(int) result,
							(unsigned int) adc_device_id,
							(unsigned int) channel
					);
					/* modbus_registers: set channel error */
					vMBRegInputSetChannelError(adc_device_id-1U, channel);
				}
				else
				{
					printf(
							"%d: ADC %u read error\r\n",
							(int) result,
							(unsigned int) adc_device_id
					);
					/* modbus_registers: set ADC error */
					vMBRegInputSetADCError(adc_device_id-1U);
				}

				if (
						(result == ERROR_CODE_AD4130_ILLEGAL_PARAM)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_RANGE)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_WRITE_LENGTH)
						|| (result == ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM)
						|| (result == ERROR_CODE_MEASUREMENT_ILLEGAL_RANGE)
				)
				{
					return result;
				}

				continue;
			}

			/* sensor_coeffs: get sensor curve */
			/* sensor_fit: temperature fit */
			fit_result = resistance_to_temperature(
					measured_resistance,
					sensor_coeffs_get_curve(adc_device_id, channel),
					&measured_temperature
			);
			if (fit_result != ERROR_CODE_NONE)
			{
				printf(
						"%d: ADC %u CHANNEL_%u fit error\r\n",
						(int) fit_result,
						(unsigned int) adc_device_id,
						(unsigned int) channel
				);
				/* modbus_registers: set channel error */
				vMBRegInputSetChannelError(adc_device_id-1U, channel);
				if (fit_result == ERROR_CODE_SENSOR_FIT_ILLEGAL_PARAM)
				{
					return fit_result;
				}

				continue;
			}

			/* update resistance and temperature cache */
			resistance[adc_device_id - 1U][channel] = measured_resistance;
			temperature[adc_device_id - 1U][channel] = measured_temperature;

			/* modbus_registers: update input registers */
			vMBRegInputUpdate(
					adc_device_id-1U,
					channel,
					measured_resistance,
					measured_temperature
			);
			read_count++;

			/* system_config: check measurement log state */
			if (system_config_get_measurement_log_enabled() != 0U)
			{
				printf(
						"%u-%u: R-%.4f ohm, T-%.5f K\r\n",
						(unsigned int) adc_device_id,
						(unsigned int) (channel+1U),
						(double) resistance[adc_device_id - 1U][channel],
						(double) temperature[adc_device_id - 1U][channel]
				);
			}
		}
	}

	if (read_count == 0U)
	{
		return ERROR_CODE_MEASUREMENT_NOT_READY;
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t get_sensor_channel(
		uint8_t channel,
		uint8_t *sensor_channel
)
{
	if (sensor_channel == NULL)
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM;
	}
	if (
			(channel > AD4130_REVERSE_CHANNEL_MAX)
			|| (
					(channel > AD4130_CHANNEL_MAX)
					&& (channel < AD4130_REVERSE_CHANNEL_MIN)
			)
	)
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_CHANNEL;
	}

	*sensor_channel = (channel >= AD4130_REVERSE_CHANNEL_MIN)
			? channel - AD4130_REVERSE_CHANNEL_MIN
			: channel;

	return ERROR_CODE_NONE;
}

static ErrorCode_t switch_next_channel(void)
{
	ErrorCode_t result;
	AD4130Range_t range;
	uint8_t sensor_channel;
	uint8_t next_channel;

	if (synchronized_channel < AD4130_REVERSE_CHANNEL_MIN)
	{
		next_channel = synchronized_channel + AD4130_REVERSE_CHANNEL_MIN;
	}
	else
	{
		sensor_channel = synchronized_channel - AD4130_REVERSE_CHANNEL_MIN;
		next_channel = (sensor_channel >= AD4130_CHANNEL_MAX)
				? AD4130_CHANNEL_MIN
				: sensor_channel + 1U;
	}

	result = get_sensor_channel(next_channel, &sensor_channel);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		/* ad4130: get the next sensor channel range */
		result = AD4130_Get_Range(adc_device_id, sensor_channel, &range);
		if (result == ERROR_CODE_NONE)
		{
			/* ad4130: disable the current channel */
			result = AD4130_Channel_Disable(
					adc_device_id,
					synchronized_channel
			);
		}
		if (result == ERROR_CODE_NONE)
		{
			/* ad4130: set the next channel current direction */
			result = AD4130_Set_Current_Direction(
					adc_device_id,
					next_channel
			);
		}
		if (result == ERROR_CODE_NONE)
		{
			switch (next_channel)
			{
				case 0U:
					result = AD4130_Channel_0(adc_device_id, range);
					break;

				case 1U:
					result = AD4130_Channel_1(adc_device_id, range);
					break;

				case 2U:
					result = AD4130_Channel_2(adc_device_id, range);
					break;

				case 3U:
					result = AD4130_Channel_3(adc_device_id, range);
					break;

				case 4U:
					result = AD4130_Channel_4(adc_device_id, range);
					break;

				case 5U:
					result = AD4130_Channel_5(adc_device_id, range);
					break;

				case 6U:
					result = AD4130_Channel_6(adc_device_id, range);
					break;

				case 7U:
					result = AD4130_Channel_7(adc_device_id, range);
					break;

				default:
					result = ERROR_CODE_MEASUREMENT_ILLEGAL_CHANNEL;
					break;
			}
		}
		if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u CHANNEL_%u switch error\r\n",
					(int) result,
					(unsigned int) adc_device_id,
					(unsigned int) next_channel
			);
			return result;
		}
	}

	synchronized_channel = next_channel;

	return ERROR_CODE_NONE;
}

static ErrorCode_t switch_next_channel_stad(void)
{
	ErrorCode_t result;
	AD4130Range_t range;
	uint8_t current_channel;
	uint8_t sensor_channel;
	uint8_t next_channel;
	uint8_t next_standard_channel;
	uint8_t next_measurement_channel;

	next_standard_channel = (
			stad_standard_channel < AD4130_REVERSE_CHANNEL_MIN
	) ? stad_standard_channel + AD4130_REVERSE_CHANNEL_MIN
			: AD4130_CHANNEL_MIN;

	if (stad_measurement_channel < AD4130_REVERSE_CHANNEL_MIN)
	{
		next_measurement_channel = stad_measurement_channel
				+ AD4130_REVERSE_CHANNEL_MIN;
	}
	else
	{
		sensor_channel = stad_measurement_channel
				- AD4130_REVERSE_CHANNEL_MIN;
		next_measurement_channel = (
				sensor_channel >= AD4130_CHANNEL_MAX
		) ? AD4130_CHANNEL_MIN : sensor_channel + 1U;
	}

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		if (
				(adc_device_id >= AD4130_STAD_DEVICE_ID_MIN)
				&& (adc_device_id <= AD4130_STAD_DEVICE_ID_MAX)
		)
		{
			current_channel = stad_standard_channel;
			next_channel = next_standard_channel;
		}
		else
		{
			current_channel = stad_measurement_channel;
			next_channel = next_measurement_channel;
		}

		result = get_sensor_channel(next_channel, &sensor_channel);
		if (result == ERROR_CODE_NONE)
		{
			/* ad4130: get the next sensor channel range */
			result = AD4130_Get_Range(
					adc_device_id,
					sensor_channel,
					&range
			);
		}
		if (result == ERROR_CODE_NONE)
		{
			/* ad4130: disable the current channel */
			result = AD4130_Channel_Disable(
					adc_device_id,
					current_channel
			);
		}
		if (result == ERROR_CODE_NONE)
		{
			/* ad4130: set the next channel current direction */
			result = AD4130_Set_Current_Direction(
					adc_device_id,
					next_channel
			);
		}
		if (result == ERROR_CODE_NONE)
		{
			switch (next_channel)
			{
				case 0U:
					result = AD4130_Channel_0(adc_device_id, range);
					break;

				case 1U:
					result = AD4130_Channel_1(adc_device_id, range);
					break;

				case 2U:
					result = AD4130_Channel_2(adc_device_id, range);
					break;

				case 3U:
					result = AD4130_Channel_3(adc_device_id, range);
					break;

				case 4U:
					result = AD4130_Channel_4(adc_device_id, range);
					break;

				case 5U:
					result = AD4130_Channel_5(adc_device_id, range);
					break;

				case 6U:
					result = AD4130_Channel_6(adc_device_id, range);
					break;

				case 7U:
					result = AD4130_Channel_7(adc_device_id, range);
					break;

				default:
					result = ERROR_CODE_MEASUREMENT_ILLEGAL_CHANNEL;
					break;
			}
		}
		if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u CHANNEL_%u switch error\r\n",
					(int) result,
					(unsigned int) adc_device_id,
					(unsigned int) next_channel
			);
			return result;
		}
	}

	stad_standard_channel = next_standard_channel;
	stad_measurement_channel = next_measurement_channel;

	return ERROR_CODE_NONE;
}

void sensor_read_reset_synchronized_conversion(void)
{
	synchronized_conversion_active = 0U;
	synchronized_channel = AD4130_CHANNEL_MIN;
	stad_standard_channel = AD4130_CHANNEL_MIN;
	stad_measurement_channel = AD4130_CHANNEL_MIN;
	for (uint8_t adc = 0U; adc < AD4130_DEVICE_COUNT; adc++)
	{
		for (
				uint8_t channel = 0U;
				channel < AD4130_SENSOR_CHANNEL_COUNT;
				channel++
		)
		{
			forward_resistance_valid[adc][channel] = 0U;
		}
	}
	/* ad4130_measurement: reset range transition state */
	AD4130_Reset_Range_Transition_State();
}

static ErrorCode_t recover_fifo(
		uint8_t error_adc_device_id,
		ErrorCode_t error,
		uint8_t synchronized
)
{
	ErrorCode_t result;
	uint8_t first_adc_device_id;
	uint8_t last_adc_device_id;
	uint8_t fifo_watermark;

	if (synchronized != 0U)
	{
		printf(
				"%d: ADC %u synchronized FIFO data discarded\r\n",
				(int) error,
				(unsigned int) error_adc_device_id
		);
		for (uint8_t adc_index = 0U; adc_index < AD4130_DEVICE_COUNT; adc_index++)
		{
			/* modbus_registers: set ADC error */
			vMBRegInputSetADCError(adc_index);
		}

		synchronized_conversion_active = 0U;
		/* clear stored forward measurements */
		for (uint8_t adc_index = 0U; adc_index < AD4130_DEVICE_COUNT; adc_index++)
		{
			for (
					uint8_t channel = 0U;
					channel < AD4130_SENSOR_CHANNEL_COUNT;
					channel++
			)
			{
				forward_resistance_valid[adc_index][channel] = 0U;
			}
		}
		first_adc_device_id = AD4130_DEVICE_ID_MIN;
		last_adc_device_id = AD4130_DEVICE_ID_MAX;
		fifo_watermark = 1U;
	}
	else
	{
		printf(
				"%d: ADC %u FIFO data discarded\r\n",
				(int) error,
				(unsigned int) error_adc_device_id
		);
		/* modbus_registers: set ADC error */
		vMBRegInputSetADCError(error_adc_device_id-1U);
		first_adc_device_id = error_adc_device_id;
		last_adc_device_id = error_adc_device_id;
		fifo_watermark = 4U;
	}

	for (
			uint8_t adc_device_id = first_adc_device_id;
			adc_device_id <= last_adc_device_id;
			adc_device_id++
	)
	{
		/* ad4130: disable FIFO */
		result = AD4130_FIFO_Disable(adc_device_id);
		if (result == ERROR_CODE_NONE)
		{
			/* ad4130: enable FIFO */
			result = AD4130_FIFO_Enable(
					adc_device_id,
					fifo_watermark
			);
		}
		if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u FIFO recovery error\r\n",
					(int) result,
					(unsigned int) adc_device_id
			);
			return result;
		}
	}

	return ERROR_CODE_NONE;
}
