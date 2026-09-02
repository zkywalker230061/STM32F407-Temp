#include "storage/sensor_coeffs_storage.h"

#define SENSOR_COEFFS_STORAGE_ADDRESS		0x080E0000U
#define SENSOR_COEFFS_STORAGE_SECTOR		FLASH_SECTOR_11
#define SENSOR_COEFFS_STORAGE_MAGIC			0x31464353U  /* SCF1 */
#define SENSOR_COEFFS_STORAGE_HEADER_SIZE	12U
#define SENSOR_COEFFS_STORAGE_RECORD_SIZE	360U
#define SENSOR_COEFFS_STORAGE_RECORD_COUNT	364U

#define SENSOR_COEFFS_BINARY_MIN_SIZE		36U
#define SENSOR_COEFFS_BINARY_MAX_SIZE		348U

static HAL_StatusTypeDef Sensor_Coeffs_Storage_Write_Word(
		uint32_t address,
		uint32_t data
)
{
	return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);
}

int Sensor_Coeffs_Storage_Save(
		uint8_t adc_device_id,
		uint8_t channel_id,
		const uint8_t *binary_data,
		uint32_t binary_length
)
{
	HAL_StatusTypeDef status;
	uint32_t record_address;
	uint32_t write_address;
	uint32_t write_data;
	uint32_t data_position;
	uint32_t target;

	if (
			(binary_data == NULL)
			|| (adc_device_id < 1U) || (adc_device_id > 2U)
			|| (channel_id > 3U)
	)
	{
		return SENSOR_COEFFS_STORAGE_PARAM_ERROR;
	}

	if (
			(binary_length < SENSOR_COEFFS_BINARY_MIN_SIZE)
			|| (binary_length > SENSOR_COEFFS_BINARY_MAX_SIZE)
	)
	{
		return SENSOR_COEFFS_STORAGE_LENGTH_ERROR;
	}

	record_address = SENSOR_COEFFS_STORAGE_ADDRESS;
	for (uint32_t record = 0; record < SENSOR_COEFFS_STORAGE_RECORD_COUNT; record++)
	{
		if (
				(*(const uint32_t *)record_address == 0xFFFFFFFFU)
				&& (*(const uint32_t *)(record_address + 4U) == 0xFFFFFFFFU)
				&& (*(const uint32_t *)(record_address + 8U) == 0xFFFFFFFFU)
		)
		{
			break;
		}

		record_address += SENSOR_COEFFS_STORAGE_RECORD_SIZE;
	}

	if (
			record_address >= SENSOR_COEFFS_STORAGE_ADDRESS
			+ SENSOR_COEFFS_STORAGE_RECORD_COUNT * SENSOR_COEFFS_STORAGE_RECORD_SIZE
	)
	{
		return SENSOR_COEFFS_STORAGE_FULL_ERROR;
	}

	status = HAL_FLASH_Unlock();
	if (status != HAL_OK)
	{
		return SENSOR_COEFFS_STORAGE_WRITE_ERROR;
	}

	status = Sensor_Coeffs_Storage_Write_Word(
			record_address + 4U,
			binary_length
	);
	if (status != HAL_OK)
	{
		HAL_FLASH_Lock();
		return SENSOR_COEFFS_STORAGE_WRITE_ERROR;
	}

	target = (
			(uint32_t)adc_device_id
			| ((uint32_t)channel_id << 8)
	);
	status = Sensor_Coeffs_Storage_Write_Word(
			record_address + 8U,
			target
	);
	if (status != HAL_OK)
	{
		HAL_FLASH_Lock();
		return SENSOR_COEFFS_STORAGE_WRITE_ERROR;
	}

	write_address = record_address + SENSOR_COEFFS_STORAGE_HEADER_SIZE;
	data_position = 0U;

	while (data_position < binary_length)
	{
		write_data = 0xFFFFFFFFU;

		for (uint8_t byte = 0; byte < 4U; byte++)
		{
			if (data_position < binary_length)
			{
				write_data &= ~(0xFFU << (byte * 8U));
				write_data |= (uint32_t)binary_data[data_position] << (byte * 8U);
				data_position++;
			}
		}

		status = Sensor_Coeffs_Storage_Write_Word(write_address, write_data);
		if (status != HAL_OK)
		{
			HAL_FLASH_Lock();
			return SENSOR_COEFFS_STORAGE_WRITE_ERROR;
		}

		write_address += sizeof(write_data);
	}

	if (
			*(const uint32_t *)(record_address + 4U)
			!= binary_length
	)
	{
		HAL_FLASH_Lock();
		return SENSOR_COEFFS_STORAGE_VERIFY_ERROR;
	}
	if (*(const uint32_t *)(record_address + 8U) != target)
	{
		HAL_FLASH_Lock();
		return SENSOR_COEFFS_STORAGE_VERIFY_ERROR;
	}

	for (uint32_t i = 0; i < binary_length; i++)
	{
		if (
				*(const uint8_t *)(
						record_address
						+ SENSOR_COEFFS_STORAGE_HEADER_SIZE + i
				) != binary_data[i]
		)
		{
			HAL_FLASH_Lock();
			return SENSOR_COEFFS_STORAGE_VERIFY_ERROR;
		}
	}

	status = Sensor_Coeffs_Storage_Write_Word(
			record_address,
			SENSOR_COEFFS_STORAGE_MAGIC
	);
	if (status != HAL_OK)
	{
		HAL_FLASH_Lock();
		return SENSOR_COEFFS_STORAGE_WRITE_ERROR;
	}

	HAL_FLASH_Lock();

	if (*(const uint32_t *)record_address != SENSOR_COEFFS_STORAGE_MAGIC)
	{
		return SENSOR_COEFFS_STORAGE_VERIFY_ERROR;
	}

	return SENSOR_COEFFS_STORAGE_OK;
}

int Sensor_Coeffs_Storage_Load(
		uint8_t adc_device_id,
		uint8_t channel_id,
		const uint8_t **binary_data,
		uint32_t *binary_length
)
{
	uint32_t record_address;
	uint32_t stored_length;
	uint32_t target;
	uint8_t stored_adc_device_id;
	uint8_t stored_channel_id;

	if (
			(binary_data == NULL) || (binary_length == NULL)
			|| (adc_device_id < 1U) || (adc_device_id > 2U)
			|| (channel_id > 3U)
	)
	{
		return SENSOR_COEFFS_STORAGE_PARAM_ERROR;
	}

	*binary_data = NULL;
	*binary_length = 0U;

	record_address = SENSOR_COEFFS_STORAGE_ADDRESS;
	for (uint32_t record = 0; record < SENSOR_COEFFS_STORAGE_RECORD_COUNT; record++)
	{
		if (
				(*(const uint32_t *)record_address == 0xFFFFFFFFU)
				&& (*(const uint32_t *)(record_address + 4U) == 0xFFFFFFFFU)
				&& (*(const uint32_t *)(record_address + 8U) == 0xFFFFFFFFU)
		)
		{
			break;
		}

		if (*(const uint32_t *)record_address != SENSOR_COEFFS_STORAGE_MAGIC)
		{
			record_address += SENSOR_COEFFS_STORAGE_RECORD_SIZE;
			continue;
		}

		stored_length = *(const uint32_t *)(record_address + 4U);
		if (
				(stored_length < SENSOR_COEFFS_BINARY_MIN_SIZE)
				|| (stored_length > SENSOR_COEFFS_BINARY_MAX_SIZE)
		)
		{
			record_address += SENSOR_COEFFS_STORAGE_RECORD_SIZE;
			continue;
		}

		target = *(const uint32_t *)(record_address + 8U);
		if ((target & 0xFFFF0000U) != 0U)
		{
			record_address += SENSOR_COEFFS_STORAGE_RECORD_SIZE;
			continue;
		}

		stored_adc_device_id = (uint8_t)(target & 0xFFU);
		stored_channel_id = (uint8_t)((target >> 8) & 0xFFU);
		if (
				(stored_adc_device_id == adc_device_id)
				&& (stored_channel_id == channel_id)
		)
		{
			*binary_data = (const uint8_t *)(
					record_address + SENSOR_COEFFS_STORAGE_HEADER_SIZE
			);
			*binary_length = stored_length;
		}

		record_address += SENSOR_COEFFS_STORAGE_RECORD_SIZE;
	}

	if (*binary_data == NULL)
	{
		return SENSOR_COEFFS_STORAGE_NOT_FOUND;
	}

	return SENSOR_COEFFS_STORAGE_OK;
}
