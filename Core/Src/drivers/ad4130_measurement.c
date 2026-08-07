#include "drivers/ad4130_measurement.h"

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

	return HAL_OK;
}
