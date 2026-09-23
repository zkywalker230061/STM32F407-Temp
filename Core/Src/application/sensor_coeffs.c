#include "application/sensor_coeffs.h"

#include <stddef.h>
#include <stdio.h>

#include "communication/usb_cdc/usb_cdc_scup.h"
#include "storage/sensor_coeffs_decoder.h"
#include "storage/sensor_coeffs_format.h"
#include "storage/sensor_coeffs_storage.h"


static Curve_t sensor_curves[2][4];
static Curve_t received_curve;
static uint8_t sensor_curve_valid[2][4];
static uint8_t sensor_coeffs_ram_data[2][4][SENSOR_COEFFS_BINARY_MAX_SIZE];
static uint32_t sensor_coeffs_ram_length[2][4];
static uint8_t sensor_coeffs_ram_valid[2][4];

static ErrorCode_t sensor_coeffs_copy_flash_to_ram(void);
static ErrorCode_t sensor_coeffs_copy_ram_to_flash(void);
static ErrorCode_t sensor_coeffs_compact_flash(void);


ErrorCode_t sensor_coeffs_initialize(void)
{
	const uint8_t *binary_data;
	uint32_t binary_length;
	ErrorCode_t storage_result;
	ErrorCode_t decode_result;
	ErrorCode_t result;

	result = ERROR_CODE_NONE;

	for (uint8_t adc_device_id = 1U; adc_device_id <= 2U; adc_device_id++)
	{
		for (uint8_t channel = 0U; channel < 4U; channel++)
		{
			sensor_curve_valid[adc_device_id - 1U][channel] = 0U;

			/* storage load */
			storage_result = Sensor_Coeffs_Storage_Load(
					adc_device_id,
					channel,
					&binary_data,
					&binary_length
			);
			if (storage_result == ERROR_CODE_COEFFS_STORAGE_NOT_FOUND)
			{
				continue;
			}
			if (storage_result != ERROR_CODE_NONE)
			{
				printf(
						"%d: ADC %u CHANNEL_%u storage load error\r\n",
						(int)storage_result,
						(unsigned int)adc_device_id,
						(unsigned int)channel
				);
				result = storage_result;
				continue;
			}

			/* decode */
			decode_result = Sensor_Coeffs_Decode(
					binary_data,
					binary_length,
					&sensor_curves[adc_device_id - 1U][channel]
			);
			if (decode_result != ERROR_CODE_NONE)
			{
				printf(
						"%d: ADC %u CHANNEL_%u decode error\r\n",
						(int)decode_result,
						(unsigned int)adc_device_id,
						(unsigned int)channel
				);
				result = decode_result;
				continue;
			}

			sensor_curve_valid[adc_device_id - 1U][channel] = 1U;
			printf(
					"ADC %u CHANNEL_%u: %u segments loaded\r\n",
					(unsigned int)adc_device_id,
					(unsigned int)channel,
					(unsigned int)sensor_curves[adc_device_id - 1U][channel].segment_count
			);
		}
	}

	return result;
}

ErrorCode_t sensor_coeffs_process(void)
{
	const uint8_t *binary_data;
	uint32_t binary_length;
	uint8_t adc_device_id;
	uint8_t channel;
	ErrorCode_t transfer_result;
	ErrorCode_t decode_result;
	ErrorCode_t storage_result;
	ErrorCode_t compact_result;

	/* transfer */
	transfer_result = USB_CDC_SCUP_Get_Data(
			&adc_device_id,
			&channel,
			&binary_data,
			&binary_length
	);
	if (transfer_result == ERROR_CODE_COEFFS_TRANSFER_NOT_READY)
	{
		return transfer_result;
	}
	if (transfer_result != ERROR_CODE_NONE)
	{
		printf(
				"%d: Sensor coefficients transfer error\r\n",
				(int)transfer_result
		);
		USB_CDC_SCUP_Reset();
		return transfer_result;
	}

	/* decode */
	decode_result = Sensor_Coeffs_Decode(
			binary_data,
			binary_length,
			&received_curve
	);
	if (decode_result != ERROR_CODE_NONE)
	{
		printf(
				"%d: ADC %u CHANNEL_%u decode error\r\n",
				(int)decode_result,
				(unsigned int)adc_device_id,
				(unsigned int)channel
		);
		USB_CDC_SCUP_Reset();
		return decode_result;
	}

	/* storage save */
	storage_result = Sensor_Coeffs_Storage_Save(
			adc_device_id,
			channel,
			binary_data,
			binary_length
	);
	if (storage_result == ERROR_CODE_COEFFS_STORAGE_FULL)
	{
		/* compact flash */
		compact_result = sensor_coeffs_compact_flash();
		if (compact_result != ERROR_CODE_NONE)
		{
			USB_CDC_SCUP_Reset();
			return compact_result;
		}

		storage_result = Sensor_Coeffs_Storage_Save(
				adc_device_id,
				channel,
				binary_data,
				binary_length
		);
	}
	if (storage_result != ERROR_CODE_NONE)
	{
		printf(
				"%d: ADC %u CHANNEL_%u storage save error\r\n",
				(int)storage_result,
				(unsigned int)adc_device_id,
				(unsigned int)channel
		);
		USB_CDC_SCUP_Reset();
		return storage_result;
	}

	sensor_curves[adc_device_id - 1U][channel] = received_curve;
	sensor_curve_valid[adc_device_id - 1U][channel] = 1U;
	printf(
			"ADC %u CHANNEL_%u: %u segments saved\r\n",
			(unsigned int)adc_device_id,
			(unsigned int)channel,
			(unsigned int)received_curve.segment_count
	);
	USB_CDC_SCUP_Reset();

	return ERROR_CODE_NONE;
}

const Curve_t *sensor_coeffs_get_curve(
		uint8_t adc_device_id,
		uint8_t channel
)
{
	if (
			(adc_device_id < 1U) || (adc_device_id > 2U)
			|| (channel > 3U)
	)
	{
		return NULL;
	}

	if (sensor_curve_valid[adc_device_id - 1U][channel] == 0U)
	{
		return NULL;
	}

	return &sensor_curves[adc_device_id - 1U][channel];
}

static ErrorCode_t sensor_coeffs_copy_flash_to_ram(void)
{
	const uint8_t *binary_data;
	uint32_t binary_length;
	ErrorCode_t storage_result;

	for (uint8_t adc_device_id = 1U; adc_device_id <= 2U; adc_device_id++)
	{
		for (uint8_t channel = 0U; channel < 4U; channel++)
		{
			sensor_coeffs_ram_length[adc_device_id - 1U][channel] = 0U;
			sensor_coeffs_ram_valid[adc_device_id - 1U][channel] = 0U;

			storage_result = Sensor_Coeffs_Storage_Load(
					adc_device_id,
					channel,
					&binary_data,
					&binary_length
			);
			if (storage_result == ERROR_CODE_COEFFS_STORAGE_NOT_FOUND)
			{
				continue;
			}
			if (storage_result != ERROR_CODE_NONE)
			{
				printf(
						"%d: ADC %u CHANNEL_%u storage copy to RAM error\r\n",
						(int)storage_result,
						(unsigned int)adc_device_id,
						(unsigned int)channel
				);
				return storage_result;
			}

			for (uint32_t i = 0U; i < binary_length; i++)
			{
				sensor_coeffs_ram_data[adc_device_id - 1U][channel][i]
						= binary_data[i];
			}

			sensor_coeffs_ram_length[adc_device_id - 1U][channel]
					= binary_length;
			sensor_coeffs_ram_valid[adc_device_id - 1U][channel] = 1U;
		}
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t sensor_coeffs_copy_ram_to_flash(void)
{
	ErrorCode_t storage_result;

	for (uint8_t adc_device_id = 1U; adc_device_id <= 2U; adc_device_id++)
	{
		for (uint8_t channel = 0U; channel < 4U; channel++)
		{
			if (sensor_coeffs_ram_valid[adc_device_id - 1U][channel] == 0U)
			{
				continue;
			}

			storage_result = Sensor_Coeffs_Storage_Save(
					adc_device_id,
					channel,
					sensor_coeffs_ram_data[adc_device_id - 1U][channel],
					sensor_coeffs_ram_length[adc_device_id - 1U][channel]
			);
			if (storage_result != ERROR_CODE_NONE)
			{
				printf(
						"%d: ADC %u CHANNEL_%u storage copy to FLASH error\r\n",
						(int)storage_result,
						(unsigned int)adc_device_id,
						(unsigned int)channel
				);
				return storage_result;
			}
		}
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t sensor_coeffs_compact_flash(void)
{
	ErrorCode_t result;
	ErrorCode_t storage_result;

	/* copy flash to ram */
	result = sensor_coeffs_copy_flash_to_ram();
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	/* erase flash */
	storage_result = Sensor_Coeffs_Storage_Erase();
	if (storage_result != ERROR_CODE_NONE)
	{
		printf(
				"%d: Sensor coefficients storage erase error\r\n",
				(int)storage_result
		);
		return storage_result;
	}

	/* copy ram to flash */
	result = sensor_coeffs_copy_ram_to_flash();
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	return ERROR_CODE_NONE;
}
