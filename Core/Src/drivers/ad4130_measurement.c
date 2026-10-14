#include "drivers/ad4130_measurement.h"

#include <stddef.h>

#include "common/ad4130_config_file.h"
#include "drivers/ad4130.h"


/* Gain: 8, 16, 32, 64, 128 */
#define AD4130_DATA_LOW		0x346DC6U      /* 64, 32, 16, 8, 4 mV */
#define AD4130_DATA_HIGH		0xD1B717U  /* 256,128,64,32,16 mV */

static uint8_t discard_next_samples
		[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT] = {0};
static uint8_t discard_next_fifo_batch[AD4130_DEVICE_COUNT] = {0};

static ErrorCode_t AD4130_Convert_Resistance(
		AD4130Range_t range,
		uint32_t data,
		float *resistance,
		AD4130Range_t *new_range
);
static ErrorCode_t AD4130_Get_Autorange_Range(
		AD4130Range_t range,
		uint32_t data,
		AD4130Range_t *new_range
);


ErrorCode_t AD4130_Read_Resistance(
		uint8_t adc_device_id,
		uint8_t *channel,
		float *resistance
)
{
	ErrorCode_t result;
	ErrorCode_t conversion_result;
	AD4130Range_t range;
	AD4130Range_t new_range;
	uint32_t data_status = 0U;
	uint32_t data;
	uint8_t status;

	if (
			(adc_device_id < AD4130_DEVICE_ID_MIN)
			|| (adc_device_id > AD4130_DEVICE_ID_MAX)
	)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}
	if ((channel == NULL) || (resistance == NULL))
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM;
	}
	*channel = 0xFFU;
	*resistance = 0.0f;

	/* ad4130: read conversion data */
	result = AD4130_Read_32_Bit(adc_device_id, AD4130_DATA, &data_status);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	status = data_status & 0xFFU;
	data = (data_status >> 8) & 0xFFFFFFU;

	*channel = status & 0x0FU;
	if (*channel > AD4130_CHANNEL_MAX)
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_CHANNEL;
	}

	/* ad4130: check conversion status */
	result = AD4130_Check_Status_Error(adc_device_id, status);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}
	if ((status & 0x80U) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_NOT_READY;
	}
	if (discard_next_samples[adc_device_id-1U][*channel] != 0U)
	{
		discard_next_samples[adc_device_id-1U][*channel] = 0U;
		return ERROR_CODE_MEASUREMENT_NOT_READY;
	}

	/* ad4130: get channel range */
	result = AD4130_Get_Range(adc_device_id, *channel, &range);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	conversion_result = AD4130_Convert_Resistance(
			range,
			data,
			resistance,
			&new_range
	);
	if (new_range != range)
	{
		/* ad4130: update channel range */
		result = AD4130_Set_Range(adc_device_id, *channel, new_range);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
		discard_next_samples[adc_device_id-1U][*channel] = 1U;
	}

	return conversion_result;
}

ErrorCode_t AD4130_Read_Resistance_FIFO(
		uint8_t adc_device_id,
		uint8_t *channels,
		float *resistances,
		ErrorCode_t *sample_results,
		uint8_t sample_count,
		uint8_t autorange_enabled,
		uint8_t discard_after_range_change
)
{
	ErrorCode_t result;
	AD4130FIFOSample_t samples[4U];
	AD4130Range_t ranges[AD4130_SENSOR_CHANNEL_COUNT];
	AD4130Range_t new_ranges[AD4130_SENSOR_CHANNEL_COUNT];
	AD4130Range_t new_range;
	uint8_t channel;
	uint8_t sensor_channel;

	if (
			(adc_device_id < AD4130_DEVICE_ID_MIN)
			|| (adc_device_id > AD4130_DEVICE_ID_MAX)
	)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}
	if (
			(channels == NULL)
			|| (resistances == NULL)
			|| (sample_results == NULL)
	)
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM;
	}
	if ((sample_count == 0U) || (sample_count > 4U))
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM;
	}

	/* ad4130: get channel ranges */
	for (channel = 0U; channel < AD4130_SENSOR_CHANNEL_COUNT; channel++)
	{
		result = AD4130_Get_Range(adc_device_id, channel, &ranges[channel]);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
		new_ranges[channel] = ranges[channel];
	}

	/* ad4130: read FIFO */
	result = AD4130_FIFO_Read(adc_device_id, samples, sample_count);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}
	/* discard the pending FIFO batch */
	if (discard_next_fifo_batch[adc_device_id-1U] != 0U)
	{
		discard_next_fifo_batch[adc_device_id-1U] = 0U;
		for (uint8_t sample = 0U; sample < sample_count; sample++)
		{
			channels[sample] = samples[sample].header & 0x0FU;
			resistances[sample] = 0.0f;
			sample_results[sample] = ERROR_CODE_MEASUREMENT_NOT_READY;
		}

		return ERROR_CODE_NONE;
	}

	for (uint8_t sample = 0U; sample < sample_count; sample++)
	{
		channels[sample] = samples[sample].header & 0x0FU;
		resistances[sample] = 0.0f;
		channel = channels[sample];
		if (
				(channel > AD4130_REVERSE_CHANNEL_MAX)
				|| (
						(channel > AD4130_CHANNEL_MAX)
						&& (channel < AD4130_REVERSE_CHANNEL_MIN)
				)
		)
		{
			sample_results[sample] = ERROR_CODE_MEASUREMENT_ILLEGAL_CHANNEL;
			continue;
		}
		sensor_channel = (channel >= AD4130_REVERSE_CHANNEL_MIN)
				? channel - AD4130_REVERSE_CHANNEL_MIN
				: channel;
		new_range = ranges[sensor_channel];
		sample_results[sample] = AD4130_Convert_Resistance(
				ranges[sensor_channel],
				samples[sample].data,
				&resistances[sample],
				&new_range
		);
		if (autorange_enabled != 0U)
		{
			new_ranges[sensor_channel] = new_range;
		}
	}

	/* ad4130: set range changes after processing FIFO */
	for (channel = 0U; channel < AD4130_SENSOR_CHANNEL_COUNT; channel++)
	{
		if (new_ranges[channel] == ranges[channel])
		{
			continue;
		}

		result = AD4130_Set_Range(
				adc_device_id,
				channel,
				new_ranges[channel]
		);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
		if (discard_after_range_change != 0U)
		{
			/* discard the next FIFO batch */
			discard_next_fifo_batch[adc_device_id-1U] = 1U;
		}
	}

	return ERROR_CODE_NONE;
}

void AD4130_Reset_Range_Transition_State(void)
{
	for (uint8_t adc = 0U; adc < AD4130_DEVICE_COUNT; adc++)
	{
		discard_next_fifo_batch[adc] = 0U;
		for (uint8_t channel = 0U; channel < AD4130_SENSOR_CHANNEL_COUNT; channel++)
		{
			discard_next_samples[adc][channel] = 0U;
		}
	}
}

static ErrorCode_t AD4130_Convert_Resistance(
		AD4130Range_t range,
		uint32_t data,
		float *resistance,
		AD4130Range_t *new_range
)
{
	ErrorCode_t result;
	AD4130RangeConfig_t config;
	float voltage;

	if ((resistance == NULL) || (new_range == NULL))
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM;
	}
	*resistance = 0.0f;
	*new_range = range;

	/* ad4130: get range configuration */
	result = AD4130_Get_Range_Config(range, &config);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}
	if (range == AD4130_RANGE_OFF)
	{
		return ERROR_CODE_MEASUREMENT_BELOW_RANGE;
	}
	if ((config.current <= 0.0f) || (config.gain <= 0.0f))
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_RANGE;
	}

	voltage = (float) data / 16777216.0f * AD4130_VREF / config.gain;
	*resistance = voltage / config.current;

	return AD4130_Get_Autorange_Range(range, data, new_range);
}

static ErrorCode_t AD4130_Get_Autorange_Range(
		AD4130Range_t range,
		uint32_t data,
		AD4130Range_t *new_range
)
{
	if (new_range == NULL)
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM;
	}
	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_RANGE;
	}
	*new_range = range;
	if (range == AD4130_RANGE_OFF)
	{
		return ERROR_CODE_MEASUREMENT_BELOW_RANGE;
	}

	data &= 0xFFFFFFU;
	if (data >= AD4130_DATA_HIGH)
	{
		if (range == AD4130_RANGE_10UA_GAIN_8)
		{
			return ERROR_CODE_MEASUREMENT_ABOVE_RANGE;
		}
		*new_range = (AD4130Range_t) (range-1U);
	}
	else if (data <= AD4130_DATA_LOW)
	{
		if (range == AD4130_RANGE_200UA_GAIN_128)
		{
			*new_range = AD4130_RANGE_OFF;
			return ERROR_CODE_MEASUREMENT_BELOW_RANGE;
		}
		*new_range = (AD4130Range_t) (range+1U);
	}

	return ERROR_CODE_NONE;
}
