#include "application/sensor_adc.h"

#include <stdio.h>

#include "stm32f4xx_hal.h"

#include "drivers/ad4130.h"


ErrorCode_t sensor_adc_initialize(void)
{
	ErrorCode_t result;
	AD4130InitResult_t init_result[2] = {0};
	uint8_t por_detected = 0U;

	for (uint8_t i = 0; i < 2U; i++)
	{
		result = AD4130_Init(i+1U, &init_result[i]);
		if (result == ERROR_CODE_MEASUREMENT_STATUS_POR)
		{
			por_detected = 1U;
			printf(
					"%d: ADC %u POR during initialization\r\n",
					(int)result,
					(unsigned int)(i+1U)
			);
		}
		else if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u initialization error\r\n",
					(int)result,
					(unsigned int)(i+1U)
			);
			return result;
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

		if (init_result[i].id != 0x05U)
		{
			printf(
					"%d: ADC %u ID mismatch - expected 0x05, received 0x%02X\r\n",
					(int)ERROR_CODE_AD4130_ID_MISMATCH,
					(unsigned int)(i+1U),
					(unsigned int)init_result[i].id
			);
			return ERROR_CODE_AD4130_ID_MISMATCH;
		}
		/* ADC_CONTROL:		0x2700 - 0010 0111 0000 0000 */
		/* IO_CONTROL:		0x0000 - 0000 0000 0000 0000 */
		/* VBIAS_CONTROL:	0x0000 - 0000 0000 0000 0000 */
		/* ERROR_EN:		0x0078 - 0000 0000 0111 1000 */
	}

	HAL_Delay(500);

	for (uint8_t i = 0; i < 2U; i++)
	{
		result = AD4130_Channel_0(i+1U, 2U);  /* I_OUT0_0 */
		if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u CHANNEL_0 setup error\r\n",
					(int)result,
					(unsigned int)(i+1U)
			);
			return result;
		}

//		result = AD4130_Channel_1(i+1U, 2U);
//		if (result != ERROR_CODE_NONE)
//		{
//			printf(
//					"%d: ADC %u CHANNEL_1 setup error\r\n",
//					(int)result,
//					(unsigned int)(i+1U)
//			);
//			return result;
//		}
//
//		result = AD4130_Channel_2(i+1U, 2U);
//		if (result != ERROR_CODE_NONE)
//		{
//			printf(
//					"%d: ADC %u CHANNEL_2 setup error\r\n",
//					(int)result,
//					(unsigned int)(i+1U)
//			);
//			return result;
//		}
//
//		result = AD4130_Channel_3(i+1U, 2U);
//		if (result != ERROR_CODE_NONE)
//		{
//			printf(
//					"%d: ADC %u CHANNEL_3 setup error\r\n",
//					(int)result,
//					(unsigned int)(i+1U)
//			);
//			return result;
//		}
	}

	HAL_Delay(500);

	if (por_detected != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_POR;
	}

	return ERROR_CODE_NONE;
}
