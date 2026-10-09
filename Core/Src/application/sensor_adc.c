#include "application/sensor_adc.h"

#include <stdio.h>

#include "stm32f4xx_hal.h"

#include "common/ad4130_config_file.h"
#include "drivers/ad4130.h"


#define SENSOR_ADC_INITIALIZE_RETRY_COUNT     3U
#define SENSOR_ADC_INITIALIZE_RETRY_DELAY_MS  100U

static ErrorCode_t initialize_once(void);


ErrorCode_t sensor_adc_initialize(void)
{
	ErrorCode_t result;

	result = initialize_once();
	if (
			(result == ERROR_CODE_NONE)
			|| (result == ERROR_CODE_MEASUREMENT_STATUS_POR)
	)
	{
		return result;
	}

	for (
			uint8_t retry = 0U;
			retry < SENSOR_ADC_INITIALIZE_RETRY_COUNT;
			retry++
	)
	{
		HAL_Delay(SENSOR_ADC_INITIALIZE_RETRY_DELAY_MS);
		result = initialize_once();
		if (
				(result == ERROR_CODE_NONE)
				|| (result == ERROR_CODE_MEASUREMENT_STATUS_POR)
		)
		{
			printf(
					"ADC initialization recovered after %u retries\r\n",
					(unsigned int) (retry+1U)
			);
			return result;
		}
	}

	printf(
			"%d: ADC initialization failed after %u retries\r\n",
			(int) result,
			(unsigned int) SENSOR_ADC_INITIALIZE_RETRY_COUNT
	);

	return result;
}

static ErrorCode_t initialize_once(void)
{
	ErrorCode_t result;
	AD4130InitResult_t init_result[AD4130_DEVICE_COUNT] = {0};
	uint32_t gain_value;
	uint32_t offset_value;
	uint8_t setup;
	uint8_t por_detected = 0U;

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		result = AD4130_Init(adc_device_id, &init_result[adc_device_id-1U]);
		if (result == ERROR_CODE_MEASUREMENT_STATUS_POR)
		{
			por_detected = 1U;
			printf(
					"%d: ADC %u POR during initialization\r\n",
					(int) result,
					(unsigned int) adc_device_id
			);
		}
		else if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u initialization error\r\n",
					(int) result,
					(unsigned int) adc_device_id
			);
			return result;
		}

		printf(
			"ADC %u\r\n"
			"ID: 0x%02X\r\n"
			"STATUS: 0x%02X\r\n"
			"ERROR: 0x%04X\r\n",
			(unsigned int) adc_device_id,
			/*ID: 0x05 - 0000 0101 */
			(unsigned int) init_result[adc_device_id-1U].id,
			/* STATUS (init): 0x90 - 1001 0000 or 0x10 - 0001 0000 */
			(unsigned int) init_result[adc_device_id-1U].status,
			/* ERROR: 0x0000 - 0000 0000 0000 0000 */
			(unsigned int) init_result[adc_device_id-1U].error
		);

		if (init_result[adc_device_id-1U].id != 0x05U)
		{
			printf(
					"%d: ADC %u ID mismatch - expected 0x05, received 0x%02X\r\n",
					(int) ERROR_CODE_AD4130_ID_MISMATCH,
					(unsigned int) adc_device_id,
					(unsigned int) init_result[adc_device_id-1U].id
			);
			return ERROR_CODE_AD4130_ID_MISMATCH;
		}
		/* ADC_CONTROL:		0x2710 - 0010 0111 0001 0000 */
		/* IO_CONTROL:		0x0000 - 0000 0000 0000 0000 */
		/* VBIAS_CONTROL:	0x0000 - 0000 0000 0000 0000 */
		/* ERROR_EN:		0x007B - 0000 0000 0111 1011 */
	}

	HAL_Delay(500);

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		result = AD4130_Channel_0(adc_device_id, 2U);
		if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u CHANNEL_0 setup error\r\n",
					(int) result,
					(unsigned int) adc_device_id
			);
			return result;
		}

		result = AD4130_Channel_1(adc_device_id, 2U);
		if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u CHANNEL_1 setup error\r\n",
					(int) result,
					(unsigned int) adc_device_id
			);
			return result;
		}

//		result = AD4130_Channel_2(adc_device_id, 2U);
//		if (result != ERROR_CODE_NONE)
//		{
//			printf(
//					"%d: ADC %u CHANNEL_2 setup error\r\n",
//					(int) result,
//					(unsigned int) adc_device_id
//			);
//			return result;
//		}
//
//		result = AD4130_Channel_3(adc_device_id, 2U);
//		if (result != ERROR_CODE_NONE)
//		{
//			printf(
//					"%d: ADC %u CHANNEL_3 setup error\r\n",
//					(int) result,
//					(unsigned int) adc_device_id
//			);
//			return result;
//		}

		for (uint8_t channel = 0U; channel < AD4130_SENSOR_CHANNEL_COUNT; channel++)
		{
			result = AD4130_Internal_Calibrate(
					adc_device_id,
					channel,
					&setup,
					&gain_value,
					&offset_value
			);
			if (result != ERROR_CODE_NONE)
			{
				printf(
						"%d: ADC %u CHANNEL_%u internal calibration error\r\n",
						(int) result,
						(unsigned int) adc_device_id,
						(unsigned int) channel
				);
				return result;
			}

			printf(
					"ADC %u SETUP_%u internal calibration completed: "
					"GAIN=0x%06lX, OFFSET=0x%06lX\r\n",
					(unsigned int) adc_device_id,
					(unsigned int) setup,
					(unsigned long) gain_value,
					(unsigned long) offset_value
			);
		}
	}

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		result = AD4130_FIFO_Enable(
				adc_device_id,
				AD4130_SENSOR_CHANNEL_COUNT
		);
		if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u FIFO setup error\r\n",
					(int) result,
					(unsigned int) adc_device_id
			);
			return result;
		}

		result = AD4130_Set_Conversion_Mode(
				adc_device_id,
				AD4130_CONVERSION_MODE_NORMAL
		);
		if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u conversion mode setup error\r\n",
					(int) result,
					(unsigned int) adc_device_id
			);
			return result;
		}
	}

	if (por_detected != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_POR;
	}

	return ERROR_CODE_NONE;
}
