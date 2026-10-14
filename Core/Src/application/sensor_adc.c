#include "application/sensor_adc.h"

#include <stdio.h>

#include "stm32f4xx_hal.h"

#include "common/ad4130_config_file.h"
#include "common/system_config.h"
#include "drivers/ad4130.h"
#include "application/usb_comm.h"


#define SENSOR_ADC_INITIALIZE_RETRY_COUNT     3U
#define SENSOR_ADC_INITIALIZE_RETRY_DELAY_MS  100U

static uint8_t sensor_adc_initialized;

static ErrorCode_t initialize_once(void);


ErrorCode_t sensor_adc_initialize(void)
{
	ErrorCode_t result;

	sensor_adc_initialized = 0U;

	/* initialize adc */
	result = initialize_once();
	/* usb_comm: process communication */
	(void) usb_comm_process();
	if (
			(result == ERROR_CODE_NONE)
			|| (result == ERROR_CODE_MEASUREMENT_STATUS_POR)
	)
	{
		/* system_config: apply measurement mode */
		result = system_config_apply_current_measurement_mode();
		if (result == ERROR_CODE_NONE)
		{
			sensor_adc_initialized = 1U;
			printf("initialized - ADC(s)\r\n");
			return ERROR_CODE_NONE;
		}
		printf("%d: ADC measurement mode setup error\r\n", (int) result);
	}

	/* error - retry initialize */
	for (
			uint8_t retry = 0U;
			retry < SENSOR_ADC_INITIALIZE_RETRY_COUNT;
			retry++
	)
	{
		HAL_Delay(SENSOR_ADC_INITIALIZE_RETRY_DELAY_MS);
		result = initialize_once();
		/* usb_comm: process communication */
		(void) usb_comm_process();
		if (
				(result == ERROR_CODE_NONE)
				|| (result == ERROR_CODE_MEASUREMENT_STATUS_POR)
		)
		{
			/* system_config: apply measurement mode */
			result = system_config_apply_current_measurement_mode();
			if (result == ERROR_CODE_NONE)
			{
				sensor_adc_initialized = 1U;
				printf("initialized - ADC(s)\r\n");
				printf(
						"ADC initialization recovered after %u retries\r\n",
						(unsigned int) (retry+1U)
				);
				return ERROR_CODE_NONE;
			}
			printf("%d: ADC measurement mode setup error\r\n", (int) result);
		}
	}

	printf(
			"%d: ADC initialization failed after %u retries\r\n",
			(int) result,
			(unsigned int) SENSOR_ADC_INITIALIZE_RETRY_COUNT
	);

	return result;
}

uint8_t sensor_adc_is_initialized(void)
{
	return sensor_adc_initialized;
}

static ErrorCode_t initialize_once(void)
{
	ErrorCode_t result;
	AD4130InitResult_t init_result[AD4130_DEVICE_COUNT] = {0};
	AD4130CalibrationResult_t calibration_results
			[AD4130_DEVICE_COUNT][AD4130_CALIBRATION_GAIN_COUNT] = {0};
	uint8_t calibration_error_adc_device_id = 0U;
	uint8_t por_detected = 0U;

	/* ad4130: adc initialize */
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
			"ADC %u ID: 0x%02X STATUS: 0x%02X ERROR: 0x%04X\r\n",
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
		/* ADC_CONTROL:		0x0710 - 0000 0111 0001 0000 */
		/* IO_CONTROL:		0x000F - 0000 0000 0000 1111 */
		/* VBIAS_CONTROL:	0x0000 - 0000 0000 0000 0000 */
		/* ERROR_EN:		0x007B - 0000 0000 0111 1011 */
		/* MISC:			0x2000 - 0010 0000 0000 0000 */
	}

	HAL_Delay(500);

	/* ad4130: channel initialize */
	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		result = AD4130_Channel_0(
				adc_device_id,
				AD4130_RANGE_10UA_GAIN_8
		);
		if (result != ERROR_CODE_NONE)
		{
			printf(
					"%d: ADC %u CHANNEL_0 setup error\r\n",
					(int) result,
					(unsigned int) adc_device_id
			);
			return result;
		}
	}

	/* ad4130: gain internal calibration */
	result = AD4130_Internal_Calibrate(
			calibration_results,
			&calibration_error_adc_device_id
	);
	if (result != ERROR_CODE_NONE)
	{
		printf(
				"%d: ADC %u internal calibration error\r\n",
				(int) result,
				(unsigned int) calibration_error_adc_device_id
		);
		return result;
	}

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		for (
				uint8_t gain_index = 0U;
				gain_index < AD4130_CALIBRATION_GAIN_COUNT;
				gain_index++
		)
		{
			AD4130CalibrationResult_t *calibration_result =
					&calibration_results[adc_device_id-1U][gain_index];

			printf(
					"ADC %u gain %u calibrated: "
					"GAIN=0x%06lX, OFFSET=0x%06lX\r\n",
					(unsigned int) adc_device_id,
					(unsigned int) calibration_result->gain,
					(unsigned long) calibration_result->gain_value,
					(unsigned long) calibration_result->offset_value
			);
		}
	}

	if (por_detected != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_POR;
	}

	return ERROR_CODE_NONE;
}
