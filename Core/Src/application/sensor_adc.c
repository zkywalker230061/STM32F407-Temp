#include "application/sensor_adc.h"

int sensor_adc_initialize(void)
{
	HAL_StatusTypeDef result;
	AD4130InitResult_t init_result[2] = {0};

	for (uint8_t i = 0; i < 2U; i++)
	{
		result = AD4130_Init(i+1U, &init_result[i]);
		if (result != HAL_OK)
		{
			printf(
					"ADC %u INIT incorrect: %d\r\n",
					(unsigned int)(i+1U),
					(int)result
			);
			return SENSOR_ADC_INIT_ERROR;
		}

		printf(
			"ADC %u\r\n"
			"ID: 0x%02X\r\n"
			"STATUS: 0x%02X\r\n"
			"ERROR: 0x%04X\r\n",
			(unsigned int)(i+1U),
			/*ID: 0x05 - 0000 0101 */
			(unsigned int)init_result[i].id,
			/* STATUS (init): 0x90 - 1001 0000 or 0x10 - 0001 0000 */
			(unsigned int)init_result[i].status,
			/* ERROR: 0x0000 - 0000 0000 0000 0000 */
			(unsigned int)init_result[i].error
		);

		if ((init_result[i].id != 0x05U) || (init_result[i].error != 0x0000U))
		{
			printf("ADC %u ID or ERROR incorrect\r\n", (unsigned int)(i+1U));
			return SENSOR_ADC_ID_ERROR;
		}

		/* ADC_CONTROL:		0x2700 - 0010 0111 0000 0000 */
		/* IO_CONTROL:		0x0000 - 0000 0000 0000 0000 */
		/* VBIAS_CONTROL:	0x0000 - 0000 0000 0000 0000 */
		/* ERROR_EN:		0x0078 - 0000 0000 0111 1000 */
	}

	for (uint8_t i = 0; i < 2U; i++)
	{
		result = AD4130_Channel_0(i+1U, 2U);  /* I_OUT0_0 */
		if (result != HAL_OK)
		{
			printf(
					"ADC %u CHANNEL_0 setup incorrect: %d\r\n",
					(unsigned int)(i+1U),
					(int)result
			);
			return SENSOR_ADC_SETUP_ERROR;
		}

//		result = AD4130_Channel_1(i+1U, 2U);
//		if (result != HAL_OK)
//		{
//			printf(
//					"ADC %u CHANNEL_1 setup incorrect: %d\r\n",
//					(unsigned int)(i+1U),
//					(int)result
//			);
//			return SENSOR_ADC_SETUP_ERROR;
//		}
//
//		result = AD4130_Channel_2(i+1U, 2U);
//		if (result != HAL_OK)
//		{
//			printf(
//					"ADC %u CHANNEL_2 setup incorrect: %d\r\n",
//					(unsigned int)(i+1U),
//					(int)result
//			);
//			return SENSOR_ADC_SETUP_ERROR;
//		}
//
//		result = AD4130_Channel_3(i+1U, 2U);
//		if (result != HAL_OK)
//		{
//			printf(
//					"ADC %u CHANNEL_3 setup incorrect: %d\r\n",
//					(unsigned int)(i+1U),
//					(int)result
//			);
//			return SENSOR_ADC_SETUP_ERROR;
//		}
	}

	return SENSOR_ADC_OK;
}
