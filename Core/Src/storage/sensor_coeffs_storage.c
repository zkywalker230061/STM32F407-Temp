#include "storage/sensor_coeffs_storage.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"

#include "storage/sensor_coeffs_format.h"


#define SENSOR_COEFFS_STORAGE_ADDRESS		0x080E0000U
#define SENSOR_COEFFS_STORAGE_SECTOR		FLASH_SECTOR_11
#define SENSOR_COEFFS_STORAGE_MAGIC			0x31464353U  /* SCF1 */
#define SENSOR_COEFFS_STORAGE_HEADER_SIZE	12U
#define SENSOR_COEFFS_STORAGE_RECORD_SIZE	360U
#define SENSOR_COEFFS_STORAGE_RECORD_COUNT	364U


static HAL_StatusTypeDef Sensor_Coeffs_Storage_Write_Word(
		uint32_t address,
		uint32_t data
)
{
	return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);
}

ErrorCode_t Sensor_Coeffs_Storage_Save(
		uint8_t adc_device_id,
		uint8_t channel,
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
			|| (channel > 3U)
	)
	{
		return ERROR_CODE_COEFFS_STORAGE_ILLEGAL_PARAM;
	}

	if (
			(binary_length < SENSOR_COEFFS_BINARY_MIN_SIZE)
			|| (binary_length > SENSOR_COEFFS_BINARY_MAX_SIZE)
	)
	{
		return ERROR_CODE_COEFFS_STORAGE_ILLEGAL_LENGTH;
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
		return ERROR_CODE_COEFFS_STORAGE_FULL;
	}

	status = HAL_FLASH_Unlock();
	if (status != HAL_OK)
	{
		return ERROR_CODE_COEFFS_STORAGE_WRITE;
	}

	status = Sensor_Coeffs_Storage_Write_Word(
			record_address + 4U,
			binary_length
	);
	if (status != HAL_OK)
	{
		HAL_FLASH_Lock();
		return ERROR_CODE_COEFFS_STORAGE_WRITE;
	}

	target = (
			(uint32_t)adc_device_id
			| ((uint32_t)channel << 8)
	);
	status = Sensor_Coeffs_Storage_Write_Word(
			record_address + 8U,
			target
	);
	if (status != HAL_OK)
	{
		HAL_FLASH_Lock();
		return ERROR_CODE_COEFFS_STORAGE_WRITE;
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
			return ERROR_CODE_COEFFS_STORAGE_WRITE;
		}

		write_address += sizeof(write_data);
	}

	if (
			*(const uint32_t *)(record_address + 4U)
			!= binary_length
	)
	{
		HAL_FLASH_Lock();
		return ERROR_CODE_COEFFS_STORAGE_VERIFY;
	}
	if (*(const uint32_t *)(record_address + 8U) != target)
	{
		HAL_FLASH_Lock();
		return ERROR_CODE_COEFFS_STORAGE_VERIFY;
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
			return ERROR_CODE_COEFFS_STORAGE_VERIFY;
		}
	}

	status = Sensor_Coeffs_Storage_Write_Word(
			record_address,
			SENSOR_COEFFS_STORAGE_MAGIC
	);
	if (status != HAL_OK)
	{
		HAL_FLASH_Lock();
		return ERROR_CODE_COEFFS_STORAGE_WRITE;
	}

	HAL_FLASH_Lock();

	if (*(const uint32_t *)record_address != SENSOR_COEFFS_STORAGE_MAGIC)
	{
		return ERROR_CODE_COEFFS_STORAGE_VERIFY;
	}

	return ERROR_CODE_NONE;
}

ErrorCode_t Sensor_Coeffs_Storage_Load(
		uint8_t adc_device_id,
		uint8_t channel,
		const uint8_t **binary_data,
		uint32_t *binary_length
)
{
	uint32_t record_address;
	uint32_t stored_length;
	uint32_t target;
	uint8_t stored_adc_device_id;
	uint8_t stored_channel;

	if (
			(binary_data == NULL) || (binary_length == NULL)
			|| (adc_device_id < 1U) || (adc_device_id > 2U) || (channel > 3U)
	)
	{
		return ERROR_CODE_COEFFS_STORAGE_ILLEGAL_PARAM;
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
		stored_channel = (uint8_t)((target >> 8) & 0xFFU);
		if (
				(stored_adc_device_id == adc_device_id)
				&& (stored_channel == channel)
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
		return ERROR_CODE_COEFFS_STORAGE_NOT_FOUND;
	}

	return ERROR_CODE_NONE;
}

ErrorCode_t Sensor_Coeffs_Storage_Erase(void)
{
	FLASH_EraseInitTypeDef erase_init;
	HAL_StatusTypeDef status;
	uint32_t sector_error;

	erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase_init.Sector = SENSOR_COEFFS_STORAGE_SECTOR;
	erase_init.NbSectors = 1U;
	erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	status = HAL_FLASH_Unlock();
	if (status != HAL_OK)
	{
		return ERROR_CODE_COEFFS_STORAGE_ERASE;
	}

	status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
	HAL_FLASH_Lock();

	if (status != HAL_OK)
	{
		return ERROR_CODE_COEFFS_STORAGE_ERASE;
	}

	return ERROR_CODE_NONE;
}
