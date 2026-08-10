#include "drivers/ad4130_measurement.h"

#define AD4130_DATA_LOW           0x346DC6U  /* 2.0 mV */
#define AD4130_DATA_HIGH          0xD1B717U  /* 8.0 mV */
#define AD4130_DATA_100NA_RETURN  0x01F751U  /* 7.5 mV at 10 µA */
                                             /* 75 µV at 100 nA */

static HAL_StatusTypeDef AD4130_Get_Autorange_Level(
		uint8_t current_level,
		uint32_t data,
		uint8_t *new_level
);

HAL_StatusTypeDef AD4130_Read_Channel_0_Resistance(
		uint8_t adc_device_id,
		float *resistance
)
{
	HAL_StatusTypeDef result;
	uint32_t data_status = 0;
	uint32_t data = 0;
	uint8_t status = 0;
	uint8_t channel;
	float iout;
	float voltage;
	uint8_t iout_level;
	uint8_t new_iout_level;

	if ((adc_device_id < 1U) || (adc_device_id > 2U) || (resistance == NULL))
	{
		return HAL_ERROR;
	}
	*resistance = 0.0f;

	result = AD4130_Read_32_Bit(adc_device_id, AD4130_DATA, &data_status);
	if (result != HAL_OK)
	{
		return result;
	}

	status = data_status & 0xFFU;
	data = (data_status >> 8) & 0xFFFFFFU;
	if ((status & 0x80U) != 0U)
	{
		return HAL_BUSY;
	}
	if ((status & 0x70U) != 0U)
	{
		return HAL_ERROR;
	}

	channel = status & 0x0FU;
	if (channel != 0U)
	{
		return HAL_ERROR;
	}

	iout = ad4130_iouts[adc_device_id - 1U].i_1;
	if (iout <= 0.0f)
	{
		return HAL_ERROR;
	}

	voltage = (
		(float)data / 16777216.0f
		* AD4130_VREF / AD4130_GAIN
	);

	*resistance = voltage / iout;

	iout_level = ad4130_iouts[adc_device_id - 1U].level_1;
	new_iout_level = iout_level;
	result = AD4130_Get_Autorange_Level(iout_level, data, &new_iout_level);
	if (result != HAL_OK)
	{
		return result;
	}

	if (new_iout_level != iout_level)
	{
		result = AD4130_Channel_0(adc_device_id, new_iout_level);
		if (result != HAL_OK)
		{
			return result;
		}
	}

	if (new_iout_level == 0U)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

static HAL_StatusTypeDef AD4130_Get_Autorange_Level(
		uint8_t current_level,
		uint32_t data,
		uint8_t *new_level
)
{
	if ((current_level < 1U) || (current_level > 7U) || (new_level == NULL))
	{
		return HAL_ERROR;
	}

	data &= 0xFFFFFFU;
	*new_level = current_level;

	/*
	 * 100 nA is a special level.
	 */
	if (current_level == 1U)
	{
		if (data <= AD4130_DATA_100NA_RETURN)
		{
			*new_level = 2U;  /* 100 nA -> 10 µA */
		}
		return HAL_OK;
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
				return HAL_ERROR;
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
				return HAL_ERROR;
		}
	}
	return HAL_OK;
}
