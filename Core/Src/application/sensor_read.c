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
static uint8_t synchronized_conversion_active = 0U;

static ErrorCode_t recover_fifo(
		uint8_t error_adc_device_id,
		ErrorCode_t error,
		uint8_t synchronized
);


ErrorCode_t read_sensor(void)
{
	ErrorCode_t result;
	ErrorCode_t recovery_result;
	ErrorCode_t fit_result;
	ErrorCode_t sample_results[AD4130_SENSOR_CHANNEL_COUNT];
	uint8_t channels[AD4130_SENSOR_CHANNEL_COUNT];
	float resistances[AD4130_SENSOR_CHANNEL_COUNT];
	uint8_t fifo_ready[AD4130_DEVICE_COUNT] = {0U};
	uint8_t synchronized;
	uint8_t channel;
	float measured_resistance;
	float measured_temperature;
	uint8_t read_count = 0U;

	synchronized = system_config_synchronized_measurement_enabled();
	if ((synchronized != 0U) && (synchronized_conversion_active == 0U))
	{
		/* Start synchronized sequence */
		AD4130_Synchronize();
		synchronized_conversion_active = 1U;

		return ERROR_CODE_MEASUREMENT_NOT_READY;
	}

	/* Check FIFO ready */
	for (uint8_t adc_device_id = AD4130_DEVICE_ID_MIN; adc_device_id <= AD4130_DEVICE_ID_MAX; adc_device_id++)
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
					synchronized
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
				if (synchronized != 0U)
				{
					return ERROR_CODE_MEASUREMENT_NOT_READY;
				}

				continue;
			}

			return result;
		}
		if ((fifo_ready[adc_device_id-1U] == 0U) && (synchronized != 0U))
		{
			return ERROR_CODE_MEASUREMENT_NOT_READY;
		}
	}

	if (synchronized != 0U)
	{
		synchronized_conversion_active = 0U;
	}

	for (uint8_t adc_device_id = AD4130_DEVICE_ID_MIN; adc_device_id <= AD4130_DEVICE_ID_MAX; adc_device_id++)
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
				AD4130_SENSOR_CHANNEL_COUNT
		);
		if (result != ERROR_CODE_NONE)
		{
			recovery_result = recover_fifo(
					adc_device_id,
					result,
					synchronized
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
				if (synchronized != 0U)
				{
					return ERROR_CODE_MEASUREMENT_NOT_READY;
				}

				continue;
			}

			return result;
		}

		for (uint8_t sample = 0U; sample < AD4130_SENSOR_CHANNEL_COUNT; sample++)
		{
			channel = channels[sample];
			measured_resistance = resistances[sample];
			result = sample_results[sample];
			if (result != ERROR_CODE_NONE)
			{
				if (
						(channel <= AD4130_CHANNEL_MAX)
						&& (
								(result == ERROR_CODE_MEASUREMENT_ILLEGAL_IOUT)
								|| (result == ERROR_CODE_MEASUREMENT_BELOW_RANGE)
								|| (result == ERROR_CODE_MEASUREMENT_ABOVE_RANGE)
						)
				)
				{
					printf(
							"%d: ADC %u CHANNEL_%u read error\r\n",
							(int) result,
							(unsigned int) adc_device_id,
							(unsigned int) channel
					);
					vMBRegInputSetChannelError(adc_device_id-1U, channel);
				}
				else
				{
					printf(
							"%d: ADC %u read error\r\n",
							(int) result,
							(unsigned int) adc_device_id
					);
					vMBRegInputSetADCError(adc_device_id-1U);
				}

				if (
						(result == ERROR_CODE_AD4130_ILLEGAL_PARAM)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_IOUT)
						|| (result == ERROR_CODE_AD4130_ILLEGAL_WRITE_LENGTH)
						|| (result == ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM)
				)
				{
					return result;
				}

				continue;
			}

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
				vMBRegInputSetChannelError(adc_device_id-1U, channel);
				if (fit_result == ERROR_CODE_SENSOR_FIT_ILLEGAL_PARAM)
				{
					return fit_result;
				}

				continue;
			}

			/* Update resistance and temperature arrays, and Modbus registers */
			resistance[adc_device_id - 1U][channel] = measured_resistance;
			temperature[adc_device_id - 1U][channel] = measured_temperature;
			vMBRegInputUpdate(
					adc_device_id-1U,
					channel,
					measured_resistance,
					measured_temperature
			);
			read_count++;

			/* usb comm measurement log */
			if (system_config_measurement_log_enabled() != 0U)
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

void sensor_read_reset_synchronized_conversion(void)
{
	synchronized_conversion_active = 0U;
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

	if (synchronized != 0U)
	{
		printf(
				"%d: ADC %u synchronized FIFO data discarded\r\n",
				(int) error,
				(unsigned int) error_adc_device_id
		);
		for (uint8_t adc_index = 0U; adc_index < AD4130_DEVICE_COUNT; adc_index++)
		{
			vMBRegInputSetADCError(adc_index);
		}

		synchronized_conversion_active = 0U;
		first_adc_device_id = AD4130_DEVICE_ID_MIN;
		last_adc_device_id = AD4130_DEVICE_ID_MAX;
	}
	else
	{
		printf(
				"%d: ADC %u FIFO data discarded\r\n",
				(int) error,
				(unsigned int) error_adc_device_id
		);
		vMBRegInputSetADCError(error_adc_device_id-1U);
		first_adc_device_id = error_adc_device_id;
		last_adc_device_id = error_adc_device_id;
	}

	for (
			uint8_t adc_device_id = first_adc_device_id;
			adc_device_id <= last_adc_device_id;
			adc_device_id++
	)
	{
		result = AD4130_FIFO_Disable(adc_device_id);
		if (result == ERROR_CODE_NONE)
		{
			result = AD4130_FIFO_Enable(
					adc_device_id,
					AD4130_SENSOR_CHANNEL_COUNT
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
