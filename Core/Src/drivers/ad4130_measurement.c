#include "drivers/ad4130_measurement.h"

#define AD4130_DATA_LOW					0x346DC6U  /* 2.0 mV */
#define AD4130_DATA_HIGH				0xD1B717U  /* 8.0 mV */
#define AD4130_DATA_100NA_RETURN		0x01F751U  /* 75 µV at 100 nA */
										           /* 7.5 mV at 10 µA */
#define AD4130_DATA_100NA_OVER_RANGE	0xF33333U  /* 95% */

static int AD4130_Get_Autorange_Level(
		uint8_t current_level,
		uint32_t data,
		uint8_t *new_level
);

int AD4130_Read_Resistance(
		uint8_t adc_device_id,
		uint8_t *channel,
		float *resistance
)
{
	HAL_StatusTypeDef result;
	int autorange_result;
	uint32_t data_status = 0;
	uint32_t data = 0;
	uint8_t status = 0;
	float iout;
	float voltage;
	uint8_t iout_level;
	uint8_t new_iout_level;

	if (
			(adc_device_id < 1U) || (adc_device_id > 2U)
			|| (channel == NULL) || (resistance == NULL)
	)
	{
		return AD4130_MEASUREMENT_PARAM_ERROR;
	}
	*channel = 0xFFU;
	*resistance = 0.0f;

	result = AD4130_Read_32_Bit(adc_device_id, AD4130_DATA, &data_status);
	if (result == HAL_TIMEOUT)
	{
		return AD4130_MEASUREMENT_TIMEOUT;
	}
	if (result != HAL_OK)
	{
		return AD4130_MEASUREMENT_COMM_ERROR;
	}

	status = data_status & 0xFFU;
	data = (data_status >> 8) & 0xFFFFFFU;
	if ((status & 0x80U) != 0U)
	{
		return AD4130_MEASUREMENT_NOT_READY;
	}
	if ((status & 0x50U) != 0U)
	{
		return AD4130_MEASUREMENT_STATUS_ERROR;
	}

	*channel = status & 0x0FU;
	if (*channel > 3U)
	{
		return AD4130_MEASUREMENT_CHANNEL_CONFIG_ERROR;
	}

	switch (*channel)
	{
		case 0U:
			iout = ad4130_iouts[adc_device_id - 1U].i_1;
			iout_level = ad4130_iouts[adc_device_id - 1U].level_1;
			break;

		case 1U:
			iout = ad4130_iouts[adc_device_id - 1U].i_2;
			iout_level = ad4130_iouts[adc_device_id - 1U].level_2;
			break;

		case 2U:
			iout = ad4130_iouts[adc_device_id - 1U].i_3;
			iout_level = ad4130_iouts[adc_device_id - 1U].level_3;
			break;

		case 3U:
			iout = ad4130_iouts[adc_device_id - 1U].i_4;
			iout_level = ad4130_iouts[adc_device_id - 1U].level_4;
			break;

		default:
			return AD4130_MEASUREMENT_CHANNEL_CONFIG_ERROR;
	}

	if (iout_level == 0U)
	{
		return AD4130_MEASUREMENT_BELOW_RANGE;
	}
	if (iout < 0.0f)
	{
		return AD4130_MEASUREMENT_IOUT_CONFIG_ERROR;
	}

	voltage = (
		(float)data / 16777216.0f
		* AD4130_VREF / AD4130_GAIN
	);

	*resistance = voltage / iout;

	new_iout_level = iout_level;
	autorange_result = AD4130_Get_Autorange_Level(
			iout_level,
			data,
			&new_iout_level
	);
	if (autorange_result != AD4130_MEASUREMENT_OK)
	{
		return autorange_result;
	}

	if (new_iout_level != iout_level)
	{
		switch (*channel)
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
				return AD4130_MEASUREMENT_CHANNEL_CONFIG_ERROR;
		}

		if (result != HAL_OK)
		{
			if (result == HAL_TIMEOUT)
			{
				return AD4130_MEASUREMENT_TIMEOUT;
			}
			return AD4130_MEASUREMENT_COMM_ERROR;
		}
	}

	if (new_iout_level == 0U)
	{
		return AD4130_MEASUREMENT_BELOW_RANGE;
	}

	return AD4130_MEASUREMENT_OK;
}

static int AD4130_Get_Autorange_Level(
		uint8_t current_level,
		uint32_t data,
		uint8_t *new_level
)
{
	if ((current_level < 1U) || (current_level > 7U) || (new_level == NULL))
	{
		return AD4130_MEASUREMENT_IOUT_CONFIG_ERROR;
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
			return AD4130_MEASUREMENT_ABOVE_RANGE;
		}
		if (data <= AD4130_DATA_100NA_RETURN)
		{
			*new_level = 2U;  /* 100 nA -> 10 µA */
		}
		return AD4130_MEASUREMENT_OK;
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
				return AD4130_MEASUREMENT_IOUT_CONFIG_ERROR;
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
				return AD4130_MEASUREMENT_IOUT_CONFIG_ERROR;
		}
	}
	return AD4130_MEASUREMENT_OK;
}
