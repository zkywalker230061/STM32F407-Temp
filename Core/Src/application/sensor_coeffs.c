#include "application/sensor_coeffs.h"

static Curve_t sensor_curves[2][4];
static Curve_t received_curve;
static uint8_t sensor_curve_valid[2][4];
static uint8_t sensor_coeffs_ram_data[2][4][SENSOR_COEFFS_BINARY_MAX_SIZE];
static uint32_t sensor_coeffs_ram_length[2][4];
static uint8_t sensor_coeffs_ram_valid[2][4];

static int sensor_coeffs_copy_flash_to_ram(void);
static int sensor_coeffs_copy_ram_to_flash(void);
static int sensor_coeffs_compact_flash(void);

int sensor_coeffs_initialize(void)
{
	const uint8_t *binary_data;
	uint32_t binary_length;
	int storage_result;
	int decode_result;
	int result;

	result = SENSOR_COEFFS_OK;

	for (uint8_t adc_device_id = 1U; adc_device_id <= 2U; adc_device_id++)
	{
		for (uint8_t channel_id = 0U; channel_id < 4U; channel_id++)
		{
			sensor_curve_valid[adc_device_id - 1U][channel_id] = 0U;

			/* storage load */
			storage_result = Sensor_Coeffs_Storage_Load(
					adc_device_id,
					channel_id,
					&binary_data,
					&binary_length
			);
			if (storage_result == SENSOR_COEFFS_STORAGE_NOT_FOUND)
			{
				continue;
			}
			if (storage_result != SENSOR_COEFFS_STORAGE_OK)
			{
				printf(
						"ADC %u CHANNEL_%u storage load error: %d\r\n",
						(unsigned int)adc_device_id,
						(unsigned int)channel_id,
						storage_result
				);
				result = SENSOR_COEFFS_STORAGE_LOAD_ERROR;
				continue;
			}

			/* decode */
			decode_result = Sensor_Coeffs_Decode(
					binary_data,
					binary_length,
					&sensor_curves[adc_device_id - 1U][channel_id]
			);
			if (decode_result != SENSOR_COEFFS_DECODE_OK)
			{
				printf(
						"ADC %u CHANNEL_%u decode error: %d\r\n",
						(unsigned int)adc_device_id,
						(unsigned int)channel_id,
						decode_result
				);
				result = SENSOR_COEFFS_DECODE_ERROR;
				continue;
			}

			sensor_curve_valid[adc_device_id - 1U][channel_id] = 1U;
			printf(
					"ADC %u CHANNEL_%u: %u segments loaded\r\n",
					(unsigned int)adc_device_id,
					(unsigned int)channel_id,
					(unsigned int)sensor_curves[adc_device_id - 1U][channel_id].segment_count
			);
		}
	}

	return result;
}

int sensor_coeffs_process(void)
{
	const uint8_t *binary_data;
	uint32_t binary_length;
	uint8_t adc_device_id;
	uint8_t channel_id;
	int transfer_result;
	int decode_result;
	int storage_result;
	int compact_result;

	/* transfer */
	transfer_result = Sensor_Coeffs_Transfer_Get_Data(
			&adc_device_id,
			&channel_id,
			&binary_data,
			&binary_length
	);
	if (transfer_result == SENSOR_COEFFS_TRANSFER_NOT_READY)
	{
		return SENSOR_COEFFS_NOT_READY;
	}
	if (transfer_result != SENSOR_COEFFS_TRANSFER_OK)
	{
		printf(
				"Sensor coefficients transfer error: %d\r\n",
				transfer_result
		);
		Sensor_Coeffs_Transfer_Reset();
		return SENSOR_COEFFS_TRANSFER_ERROR;
	}

	/* decode */
	decode_result = Sensor_Coeffs_Decode(
			binary_data,
			binary_length,
			&received_curve
	);
	if (decode_result != SENSOR_COEFFS_DECODE_OK)
	{
		printf(
				"ADC %u CHANNEL_%u decode error: %d\r\n",
				(unsigned int)adc_device_id,
				(unsigned int)channel_id,
				decode_result
		);
		Sensor_Coeffs_Transfer_Reset();
		return SENSOR_COEFFS_DECODE_ERROR;
	}

	/* storage save */
	storage_result = Sensor_Coeffs_Storage_Save(
			adc_device_id,
			channel_id,
			binary_data,
			binary_length
	);
	if (storage_result == SENSOR_COEFFS_STORAGE_FULL_ERROR)
	{
		/* compact flash */
		compact_result = sensor_coeffs_compact_flash();
		if (compact_result != SENSOR_COEFFS_OK)
		{
			Sensor_Coeffs_Transfer_Reset();
			return compact_result;
		}

		storage_result = Sensor_Coeffs_Storage_Save(
				adc_device_id,
				channel_id,
				binary_data,
				binary_length
		);
	}
	if (storage_result != SENSOR_COEFFS_STORAGE_OK)
	{
		printf(
				"ADC %u CHANNEL_%u storage save error: %d\r\n",
				(unsigned int)adc_device_id,
				(unsigned int)channel_id,
				storage_result
		);
		Sensor_Coeffs_Transfer_Reset();
		return SENSOR_COEFFS_STORAGE_SAVE_ERROR;
	}

	sensor_curves[adc_device_id - 1U][channel_id] = received_curve;
	sensor_curve_valid[adc_device_id - 1U][channel_id] = 1U;
	printf(
			"ADC %u CHANNEL_%u: %u segments saved\r\n",
			(unsigned int)adc_device_id,
			(unsigned int)channel_id,
			(unsigned int)received_curve.segment_count
	);
	Sensor_Coeffs_Transfer_Reset();

	return SENSOR_COEFFS_UPDATED;
}

const Curve_t *sensor_coeffs_get_curve(
		uint8_t adc_device_id,
		uint8_t channel_id
)
{
	if (
			(adc_device_id < 1U) || (adc_device_id > 2U)
			|| (channel_id > 3U)
	)
	{
		return NULL;
	}

	if (sensor_curve_valid[adc_device_id - 1U][channel_id] == 0U)
	{
		return NULL;
	}

	return &sensor_curves[adc_device_id - 1U][channel_id];
}

static int sensor_coeffs_copy_flash_to_ram(void)
{
	const uint8_t *binary_data;
	uint32_t binary_length;
	int storage_result;

	for (uint8_t adc_device_id = 1U; adc_device_id <= 2U; adc_device_id++)
	{
		for (uint8_t channel_id = 0U; channel_id < 4U; channel_id++)
		{
			sensor_coeffs_ram_length[adc_device_id - 1U][channel_id] = 0U;
			sensor_coeffs_ram_valid[adc_device_id - 1U][channel_id] = 0U;

			storage_result = Sensor_Coeffs_Storage_Load(
					adc_device_id,
					channel_id,
					&binary_data,
					&binary_length
			);
			if (storage_result == SENSOR_COEFFS_STORAGE_NOT_FOUND)
			{
				continue;
			}
			if (storage_result != SENSOR_COEFFS_STORAGE_OK)
			{
				printf(
						"ADC %u CHANNEL_%u storage copy to RAM error: %d\r\n",
						(unsigned int)adc_device_id,
						(unsigned int)channel_id,
						storage_result
				);
				return SENSOR_COEFFS_STORAGE_LOAD_ERROR;
			}

			for (uint32_t i = 0U; i < binary_length; i++)
			{
				sensor_coeffs_ram_data[adc_device_id - 1U][channel_id][i]
						= binary_data[i];
			}

			sensor_coeffs_ram_length[adc_device_id - 1U][channel_id]
					= binary_length;
			sensor_coeffs_ram_valid[adc_device_id - 1U][channel_id] = 1U;
		}
	}

	return SENSOR_COEFFS_OK;
}

static int sensor_coeffs_copy_ram_to_flash(void)
{
	int storage_result;

	for (uint8_t adc_device_id = 1U; adc_device_id <= 2U; adc_device_id++)
	{
		for (uint8_t channel_id = 0U; channel_id < 4U; channel_id++)
		{
			if (sensor_coeffs_ram_valid[adc_device_id - 1U][channel_id] == 0U)
			{
				continue;
			}

			storage_result = Sensor_Coeffs_Storage_Save(
					adc_device_id,
					channel_id,
					sensor_coeffs_ram_data[adc_device_id - 1U][channel_id],
					sensor_coeffs_ram_length[adc_device_id - 1U][channel_id]
			);
			if (storage_result != SENSOR_COEFFS_STORAGE_OK)
			{
				printf(
						"ADC %u CHANNEL_%u storage copy to FLASH error: %d\r\n",
						(unsigned int)adc_device_id,
						(unsigned int)channel_id,
						storage_result
				);
				return SENSOR_COEFFS_STORAGE_SAVE_ERROR;
			}
		}
	}

	return SENSOR_COEFFS_OK;
}

static int sensor_coeffs_compact_flash(void)
{
	int result;
	int storage_result;

	/* copy flash to ram */
	result = sensor_coeffs_copy_flash_to_ram();
	if (result != SENSOR_COEFFS_OK)
	{
		return result;
	}

	storage_result = Sensor_Coeffs_Storage_Erase();
	if (storage_result != SENSOR_COEFFS_STORAGE_OK)
	{
		printf(
				"Sensor coefficients storage erase error: %d\r\n",
				storage_result
		);
		return SENSOR_COEFFS_STORAGE_SAVE_ERROR;
	}

	result = sensor_coeffs_copy_ram_to_flash();
	if (result != SENSOR_COEFFS_OK)
	{
		return result;
	}

	return SENSOR_COEFFS_OK;
}
