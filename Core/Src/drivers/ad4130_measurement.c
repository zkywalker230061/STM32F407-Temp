#include "drivers/ad4130_measurement.h"

#include <stddef.h>

#include "common/ad4130_config_file.h"
#include "drivers/ad4130.h"


#define AD4130_DATA_LOW					0x346DC6U  /* 2.0 mV */
#define AD4130_DATA_HIGH				0xD1B717U  /* 8.0 mV */
#define AD4130_DATA_100NA_RETURN		0x01F751U  /* 75 µV at 100 nA */
										           /* 7.5 mV at 10 µA */
#define AD4130_DATA_100NA_OVER_RANGE	0xF33333U  /* 95% */

static ErrorCode_t AD4130_Convert_Resistance(
		uint8_t adc_device_id,
		uint8_t channel,
		uint32_t data,
		const AD4130Iouts_t *iouts,
		float *resistance
);
static ErrorCode_t AD4130_Get_Autorange_Level(
		uint8_t current_level,
		uint32_t data,
		uint8_t *new_level
);


ErrorCode_t AD4130_Read_Resistance(
		uint8_t adc_device_id,
		uint8_t *channel,
		float *resistance
)
{
	ErrorCode_t result;
	AD4130Iouts_t iouts_snapshot;
	uint32_t data_status = 0;
	uint32_t data = 0;
	uint8_t status = 0;

	if ((adc_device_id < AD4130_DEVICE_ID_MIN) || (adc_device_id > AD4130_DEVICE_ID_MAX))
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}
	if ((channel == NULL) || (resistance == NULL))
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM;
	}
	iouts_snapshot = ad4130_iouts[adc_device_id - 1U];
	*channel = 0xFFU;
	*resistance = 0.0f;

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

	result = AD4130_Check_Status_Error(adc_device_id, status);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}
	if ((status & 0x80U) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_NOT_READY;
	}

	return AD4130_Convert_Resistance(
			adc_device_id,
			*channel,
			data,
			&iouts_snapshot,
			resistance
	);
}

ErrorCode_t AD4130_Read_Resistance_FIFO(
		uint8_t adc_device_id,
		uint8_t *channels,
		float *resistances,
		ErrorCode_t *sample_results,
		uint8_t sample_count
)
{
	ErrorCode_t result;
	AD4130FIFOSample_t samples[AD4130_SENSOR_CHANNEL_COUNT];
	AD4130Iouts_t iouts_snapshot;

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
	if ((sample_count == 0U) || (sample_count > AD4130_SENSOR_CHANNEL_COUNT))
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_PARAM;
	}
	iouts_snapshot = ad4130_iouts[adc_device_id - 1U];

	result = AD4130_FIFO_Read(adc_device_id, samples, sample_count);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	for (uint8_t i = 0U; i < sample_count; i++)
	{
		channels[i] = samples[i].header & 0x0FU;
		resistances[i] = 0.0f;
		if (channels[i] > AD4130_CHANNEL_MAX)
		{
			sample_results[i] = ERROR_CODE_MEASUREMENT_ILLEGAL_CHANNEL;
			continue;
		}

		sample_results[i] = AD4130_Convert_Resistance(
				adc_device_id,
				channels[i],
				samples[i].data,
				&iouts_snapshot,
				&resistances[i]
		);
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t AD4130_Convert_Resistance(
		uint8_t adc_device_id,
		uint8_t channel,
		uint32_t data,
		const AD4130Iouts_t *iouts,
		float *resistance
)
{
	ErrorCode_t result;
	ErrorCode_t autorange_result;
	float iout;
	float voltage;
	uint8_t iout_level;
	uint8_t new_iout_level;

	switch (channel)
	{
		case 0U:
			iout = iouts->i_1;
			iout_level = iouts->level_1;
			break;

		case 1U:
			iout = iouts->i_2;
			iout_level = iouts->level_2;
			break;

		case 2U:
			iout = iouts->i_3;
			iout_level = iouts->level_3;
			break;

		case 3U:
			iout = iouts->i_4;
			iout_level = iouts->level_4;
			break;

		default:
			return ERROR_CODE_MEASUREMENT_ILLEGAL_CHANNEL;
	}

	if (iout_level == 0U)
	{
		return ERROR_CODE_MEASUREMENT_BELOW_RANGE;
	}
	if (iout <= 0.0f)
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_IOUT;
	}

	voltage = (float) data / 16777216.0f * AD4130_VREF / AD4130_GAIN;

	*resistance = voltage / iout;

	new_iout_level = iout_level;
	autorange_result = AD4130_Get_Autorange_Level(
			iout_level,
			data,
			&new_iout_level
	);
	if (autorange_result != ERROR_CODE_NONE)
	{
		return autorange_result;
	}

	if (new_iout_level != iout_level)
	{
		switch (channel)
		{
			case 0U:
				result = AD4130_Channel_0(adc_device_id, new_iout_level);
				break;

			case 1U:
				result = AD4130_Channel_1(adc_device_id, new_iout_level);
				break;

			case 2U:
				result = AD4130_Channel_2(adc_device_id, new_iout_level);
				break;

			case 3U:
				result = AD4130_Channel_3(adc_device_id, new_iout_level);
				break;

			default:
				return ERROR_CODE_MEASUREMENT_ILLEGAL_CHANNEL;
		}

		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
	}

	if (new_iout_level == 0U)
	{
		return ERROR_CODE_MEASUREMENT_BELOW_RANGE;
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t AD4130_Get_Autorange_Level(
		uint8_t current_level,
		uint32_t data,
		uint8_t *new_level
)
{
	if ((current_level < 1U) || (current_level > 7U) || (new_level == NULL))
	{
		return ERROR_CODE_MEASUREMENT_ILLEGAL_IOUT;
	}

	data &= 0xFFFFFFU;
	*new_level = current_level;

	/*
	 * 100 nA is a special level.
	 */
	if (current_level == 1U)
	{
		if (data >= AD4130_DATA_100NA_OVER_RANGE)
		{
			return ERROR_CODE_MEASUREMENT_ABOVE_RANGE;
		}
		if (data <= AD4130_DATA_100NA_RETURN)
		{
			*new_level = 2U;  /* 100 nA -> 10 µA */
		}
		return ERROR_CODE_NONE;
	}

	if (data >= AD4130_DATA_HIGH)
	{
		switch (current_level)
		{
			case 2U:
				*new_level = 1U;  /* 10 µA -> 100 nA */
				break;

			case 3U:
				*new_level = 2U;  /* 20 µA -> 10 µA */
				break;

			case 4U:
				*new_level = 3U;  /* 50 µA -> 20 µA */
				break;

			case 5U:
				*new_level = 4U;  /* 100 µA -> 50 µA */
				break;

			case 6U:
				*new_level = 5U;  /* 150 µA -> 100 µA */
				break;

			case 7U:
				*new_level = 5U;  /* 200 µA -> 100 µA */
				break;

			default:
				return ERROR_CODE_MEASUREMENT_ILLEGAL_IOUT;
		}
	}
	else if (data <= AD4130_DATA_LOW)
	{
		switch (current_level)
		{
			case 2U:
				*new_level = 3U;  /* 10 µA -> 20 µA */
				break;

			case 3U:
				*new_level = 4U;  /* 20 µA -> 50 µA */
				break;

			case 4U:
				*new_level = 5U;  /* 50 µA -> 100 µA */
				break;

			case 5U:
				*new_level = 7U;  /* 100 µA -> 200 µA */
				break;

			case 6U:
				*new_level = 7U;  /* 150 µA -> 200 µA */
				break;

			case 7U:
				*new_level = 0U;  /* 200 µA -> 0 µA */
				break;

			default:
				return ERROR_CODE_MEASUREMENT_ILLEGAL_IOUT;
		}
	}
	return ERROR_CODE_NONE;
}
