#include "application/sensor_read.h"

#include <stdio.h>

#include "common/ad4130_config_file.h"
#include "drivers/ad4130_measurement.h"
#include "application/sensor_coeffs.h"
#include "application/sensor_fit.h"
#include "application/usb_comm.h"
#include "communication/modbus/modbus_registers.h"


static float resistance[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT];
static float temperature[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT];


ErrorCode_t read_sensor(void)
{
	ErrorCode_t result;
	ErrorCode_t fit_result;
	uint8_t channel;
	float measured_resistance;
	float measured_temperature;
	uint8_t read_count = 0U;

	for (uint8_t adc_device_id = AD4130_DEVICE_ID_MIN; adc_device_id <= AD4130_DEVICE_ID_MAX; adc_device_id++)
	{
		/* ad4130_measurement: read resistance */
		result = AD4130_Read_Resistance(
				adc_device_id,
				&channel,
				&measured_resistance
		);
		if (result == ERROR_CODE_MEASUREMENT_NOT_READY)
		{
			continue;
		}
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
						(int)result,
						(unsigned int)adc_device_id,
						(unsigned int)channel
				);
				vMBRegInputSetChannelError(adc_device_id-1U, channel);
			}
			else
			{
				printf(
						"%d: ADC %u read error\r\n",
						(int)result,
						(unsigned int)adc_device_id
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
					(int)fit_result,
					(unsigned int)adc_device_id,
					(unsigned int)channel
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
		if (usb_comm_measurement_log_enabled() != 0U)
		{
			printf(
					"%u-%u: R-%.4f ohm, T-%.5f K\r\n",
					(unsigned int)adc_device_id,
					(unsigned int)(channel+1U),
					(double)resistance[adc_device_id - 1U][channel],
					(double)temperature[adc_device_id - 1U][channel]
			);
		}
	}

	if (read_count == 0U)
	{
		return ERROR_CODE_MEASUREMENT_NOT_READY;
	}

	return ERROR_CODE_NONE;
}
