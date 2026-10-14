#include "drivers/ad4130.h"

#include <stddef.h>

#include "gpio.h"
#include "spi.h"


#define AD4130_GAIN_CALIBRATION_TIMEOUT_MS    5000U
#define AD4130_OFFSET_CALIBRATION_TIMEOUT_MS  500U

/* ------------------------------------------------------------------------ */

typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
} AD4130Device_t;

static AD4130Device_t ad4130_devices[] = {
	{
		.hspi = &hspi3,
		.cs_port = SPI3_CS1_GPIO_Port,
		.cs_pin = SPI3_CS1_Pin
	},
	{
		.hspi = &hspi3,
		.cs_port = SPI3_CS2_GPIO_Port,
		.cs_pin = SPI3_CS2_Pin
	},
	{
		.hspi = &hspi3,
		.cs_port = SPI3_CS3_GPIO_Port,
		.cs_pin = SPI3_CS3_Pin
	},
	{
		.hspi = &hspi3,
		.cs_port = SPI3_CS4_GPIO_Port,
		.cs_pin = SPI3_CS4_Pin
	}
};

typedef struct
{
	AD4130RangeConfig_t measurement;
	uint8_t iout_config;
	uint8_t gain_config;
} AD4130RangeRegisterConfig_t;

static const AD4130RangeRegisterConfig_t ad4130_range_configs[] = {
	{{10.0e-6f, 8.0f}, 0b0001, 0b011},    /* 10 μA, gain 8 */
	{{10.0e-6f, 16.0f}, 0b0001, 0b100},   /* 10 μA, gain 16 */
	{{10.0e-6f, 32.0f}, 0b0001, 0b101},   /* 10 μA, gain 32 */
	{{10.0e-6f, 64.0f}, 0b0001, 0b110},   /* 10 μA, gain 64 */
	{{20.0e-6f, 64.0f}, 0b0010, 0b110},   /* 20 μA, gain 64 */
	{{50.0e-6f, 64.0f}, 0b0011, 0b110},   /* 50 μA, gain 64 */
	{{100.0e-6f, 64.0f}, 0b0100, 0b110},  /* 100 μA, gain 64 */
	{{200.0e-6f, 64.0f}, 0b0110, 0b110},  /* 200 μA, gain 64 */
	{{200.0e-6f, 128.0f}, 0b0110, 0b111}, /* 200 μA, gain 128 */
	{{0.0f, 8.0f}, 0b0000, 0b011}         /* 0 μA, gain 8 */
};

static const uint8_t ad4130_calibration_gain_configs
		[AD4130_CALIBRATION_GAIN_COUNT] = {
	0b011, /* gain 8 */
	0b100, /* gain 16 */
	0b101, /* gain 32 */
	0b110, /* gain 64 */
	0b111  /* gain 128 */
};

typedef struct
{
	uint32_t gain_value;
	uint32_t offset_value;
	uint8_t valid;
} AD4130CalibrationCache_t;

static AD4130CalibrationCache_t ad4130_calibration_cache
		[AD4130_DEVICE_COUNT][AD4130_CALIBRATION_GAIN_COUNT] = {0};

static AD4130Range_t ad4130_ranges
		[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT] = {0};

static ErrorCode_t AD4130_Set_Config(uint8_t adc_device_id);
static ErrorCode_t AD4130_Start_Calibration(
		uint8_t adc_device_id,
		uint8_t mode
);
static ErrorCode_t AD4130_Wait_Calibrations(
		uint32_t start_tick,
		uint32_t timeout_ms,
		ErrorCode_t timeout_error,
		uint8_t *error_adc_device_id
);
static ErrorCode_t AD4130_Get_Calibration_Gain_Index(
		uint8_t gain_config,
		uint8_t *gain_index
);
static ErrorCode_t AD4130_Load_Setup_Calibration(
		uint8_t adc_device_id,
		uint8_t setup,
		uint8_t gain_config
);
static ErrorCode_t AD4130_Load_All_Setup_Calibrations(
		uint8_t *error_adc_device_id
);
static ErrorCode_t AD4130_Restore_Channels(
		const uint32_t channel_values
				[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT * 2U],
		uint8_t *error_adc_device_id
);


/* ------------------------------------------------------------------------ */

static ErrorCode_t AD4130_Convert_HAL_Status(HAL_StatusTypeDef status)
{
	switch (status)
	{
		case HAL_OK:
			return ERROR_CODE_NONE;

		case HAL_BUSY:
			return ERROR_CODE_AD4130_SPI_BUSY;

		case HAL_TIMEOUT:
			return ERROR_CODE_AD4130_SPI_TIMEOUT;

		case HAL_ERROR:
		default:
			return ERROR_CODE_AD4130_SPI;
	}
}

static ErrorCode_t AD4130_Decode_Error(uint16_t error)
{
	if ((error & (1U << 11)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_AINP_OV_UV;
	}
	if ((error & (1U << 10)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_AINM_OV_UV;
	}
	if ((error & (1U << 9)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_REF_OV_UV;
	}
	if ((error & (1U << 8)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_REF_DETECT;
	}
	if ((error & (1U << 7)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_ADC;
	}
	if ((error & (1U << 6)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_SPI_IGNORE;
	}
	if ((error & (1U << 5)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_SPI_SCLK_COUNT;
	}
	if ((error & (1U << 4)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_SPI_READ;
	}
	if ((error & (1U << 3)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_SPI_WRITE;
	}
	if ((error & (1U << 2)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_SPI_CRC;
	}
	if ((error & (1U << 1)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_MM_CRC;
	}
	if ((error & (1U << 0)) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_ROM_CRC;
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t AD4130_Decode_FIFO_Status(
		uint8_t adc_device_id,
		uint8_t fifo_status
)
{
	ErrorCode_t result;
	uint16_t error = 0U;

	if ((fifo_status & 0x80U) != 0U)
	{
		result = AD4130_Read_16_Bit(adc_device_id, AD4130_ERROR, &error);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}

		result = AD4130_Decode_Error(error);
		if (result == ERROR_CODE_NONE)
		{
			return ERROR_CODE_MEASUREMENT_STATUS;
		}
		return result;
	}
	if ((fifo_status & 0x40U) != 0U)
	{
		return ERROR_CODE_AD4130_FIFO_WRITE;
	}
	if ((fifo_status & 0x20U) != 0U)
	{
		return ERROR_CODE_AD4130_FIFO_READ;
	}
	if ((fifo_status & 0x04U) != 0U)
	{
		return ERROR_CODE_AD4130_FIFO_OVERRUN;
	}

	return ERROR_CODE_NONE;
}

ErrorCode_t AD4130_Check_Status_Error(uint8_t adc_device_id, uint8_t status)
{
	ErrorCode_t result;
	uint16_t error = 0U;

	if (
			(adc_device_id < AD4130_DEVICE_ID_MIN)
			|| (adc_device_id > AD4130_DEVICE_ID_MAX)
	)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}

	if ((status & 0x40U) != 0U)
	{
		result = AD4130_Read_16_Bit(adc_device_id, AD4130_ERROR, &error);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}

		result = AD4130_Decode_Error(error);
		if (result == ERROR_CODE_NONE)
		{
			return ERROR_CODE_MEASUREMENT_STATUS;
		}
		return result;
	}

	if ((status & 0x10U) != 0U)
	{
		return ERROR_CODE_MEASUREMENT_STATUS_POR;
	}

	return ERROR_CODE_NONE;
}

/* ------------------------------------------------------------------------ */

static AD4130Device_t *AD4130_Get_Device(uint8_t adc_device_id)
{
	if (
			(adc_device_id < AD4130_DEVICE_ID_MIN)
			|| (adc_device_id > AD4130_DEVICE_ID_MAX)
	)
	{
		return NULL;  /* Invalid ADC device ID */
	}
	return &ad4130_devices[adc_device_id - 1U];
}

/* ------------------------------------------------------------------------ */

ErrorCode_t AD4130_Read_8_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint8_t *value
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[2] = {0};
	uint8_t rx[2] = {0};

	if (value == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	device = AD4130_Get_Device(adc_device_id);
	if (device == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}

	tx[0] = 0x40U | reg_addr;  /* 0b01000000, COMMS read */

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(device->hspi, tx, rx, 2U, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		*value = rx[1];
	}
	return AD4130_Convert_HAL_Status(status);
}

ErrorCode_t AD4130_Read_16_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint16_t *value
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[3] = {0};
	uint8_t rx[3] = {0};

	if (value == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	device = AD4130_Get_Device(adc_device_id);
	if (device == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}

	tx[0] = 0x40U | reg_addr;  /* 0b01000000, COMMS read */

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(device->hspi, tx, rx, 3U, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		*value = (((uint16_t) rx[1] << 8) | rx[2]);
	}
	return AD4130_Convert_HAL_Status(status);
}

ErrorCode_t AD4130_Read_24_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint32_t *value
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[4] = {0};
	uint8_t rx[4] = {0};

	if (value == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	device = AD4130_Get_Device(adc_device_id);
	if (device == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}

	tx[0] = 0x40U | reg_addr;  /* 0b01000000, COMMS read */

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(device->hspi, tx, rx, 4U, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		*value = (
			((uint32_t) rx[1] << 16)
			| ((uint32_t) rx[2] << 8)
			| rx[3]
		);
	}
	return AD4130_Convert_HAL_Status(status);
}

ErrorCode_t AD4130_Read_32_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint32_t *value
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[5] = {0};
	uint8_t rx[5] = {0};

	if (value == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	device = AD4130_Get_Device(adc_device_id);
	if (device == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}

	tx[0] = 0x40U | reg_addr;  /* 0b01000000, COMMS read */

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(device->hspi, tx, rx, 5U, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		*value = (
			((uint32_t) rx[1] << 24)
			| ((uint32_t) rx[2] << 16)
			| ((uint32_t) rx[3] << 8)
			| rx[4]
		);
	}
	return AD4130_Convert_HAL_Status(status);
}

ErrorCode_t AD4130_Write(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		const uint8_t *data,
		uint16_t length
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[4] = {0};

	if (data == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	device = AD4130_Get_Device(adc_device_id);
	if (device == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}

	if ((length == 0U) || (length > 3U))
	{
		return ERROR_CODE_AD4130_ILLEGAL_WRITE_LENGTH;
	}

	tx[0] = reg_addr & 0x3FU;  /*  0b00RS[5:0], COMMS write*/
	for (uint16_t i = 0; i < length; i++)
	{
		tx[i + 1U] = data[i];
	}

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(device->hspi, tx, length+1U, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		HAL_Delay(1);
	}
	return AD4130_Convert_HAL_Status(status);
}

ErrorCode_t AD4130_Reset(uint8_t adc_device_id)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[8] = {
			0xFFU, 0xFFU, 0xFFU, 0xFFU,
			0xFFU, 0xFFU, 0xFFU, 0xFFU
	};

	device = AD4130_Get_Device(adc_device_id);
	if (device == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(device->hspi, tx, 8U, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		HAL_Delay(10);
	}
	return AD4130_Convert_HAL_Status(status);
}

static ErrorCode_t AD4130_Start_Calibration(
		uint8_t adc_device_id,
		uint8_t mode
)
{
	ErrorCode_t result;
	uint16_t control_value;
	uint8_t tx[2] = {0};

	result = AD4130_Read_16_Bit(
			adc_device_id,
			AD4130_ADC_CONTROL,
			&control_value
	);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	/* Bits 5-2: MODE */
	control_value &= 0b1111111111000011U;
	control_value |= (uint16_t) mode << 2;

	tx[0] = (control_value >> 8) & 0xFFU;
	tx[1] = control_value & 0xFFU;
	return AD4130_Write(adc_device_id, AD4130_ADC_CONTROL, tx, 2U);
}

static ErrorCode_t AD4130_Wait_Calibrations(
		uint32_t start_tick,
		uint32_t timeout_ms,
		ErrorCode_t timeout_error,
		uint8_t *error_adc_device_id
)
{
	ErrorCode_t result;
	uint32_t pending_mask = (1UL << AD4130_DEVICE_COUNT) - 1UL;
	uint8_t status;

	if (error_adc_device_id == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	while (pending_mask != 0U)
	{
		for (
				uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
				adc_device_id <= AD4130_DEVICE_ID_MAX;
				adc_device_id++
		)
		{
			uint32_t device_mask = 1UL << (adc_device_id-1U);

			if ((pending_mask & device_mask) == 0U)
			{
				continue;
			}

			result = AD4130_Read_8_Bit(
					adc_device_id,
					AD4130_STATUS,
					&status
			);
			if (result != ERROR_CODE_NONE)
			{
				*error_adc_device_id = adc_device_id;
				return result;
			}
			/* Bit 7: RDYB */
			if ((status & 0x80U) == 0U)
			{
				result = AD4130_Check_Status_Error(adc_device_id, status);
				if (result != ERROR_CODE_NONE)
				{
					*error_adc_device_id = adc_device_id;
					return result;
				}
				pending_mask &= ~device_mask;
			}
		}

		if (pending_mask != 0U)
		{
			if ((HAL_GetTick()-start_tick) >= timeout_ms)
			{
				for (
						uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
						adc_device_id <= AD4130_DEVICE_ID_MAX;
						adc_device_id++
				)
				{
					if (
							(pending_mask
							& (1UL << (adc_device_id-1U))) != 0U
					)
					{
						*error_adc_device_id = adc_device_id;
						break;
					}
				}
				return timeout_error;
			}
			HAL_Delay(1U);
		}
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t AD4130_Get_Calibration_Gain_Index(
		uint8_t gain_config,
		uint8_t *gain_index
)
{
	if (gain_index == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	for (uint8_t i = 0U; i < AD4130_CALIBRATION_GAIN_COUNT; i++)
	{
		if (ad4130_calibration_gain_configs[i] == gain_config)
		{
			*gain_index = i;
			return ERROR_CODE_NONE;
		}
	}

	return ERROR_CODE_AD4130_ILLEGAL_RANGE;
}

static ErrorCode_t AD4130_Load_Setup_Calibration(
		uint8_t adc_device_id,
		uint8_t setup,
		uint8_t gain_config
)
{
	ErrorCode_t result;
	AD4130CalibrationCache_t *calibration;
	uint8_t gain_index;
	uint8_t tx[3] = {0};

	result = AD4130_Get_Calibration_Gain_Index(gain_config, &gain_index);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	calibration = &ad4130_calibration_cache
			[adc_device_id-1U][gain_index];
	if (calibration->valid == 0U)
	{
		return ERROR_CODE_NONE;
	}

	tx[0] = (calibration->gain_value >> 16) & 0xFFU;
	tx[1] = (calibration->gain_value >> 8) & 0xFFU;
	tx[2] = calibration->gain_value & 0xFFU;
	result = AD4130_Write(
			adc_device_id,
			(uint8_t) (AD4130_GAIN_0+setup),
			tx,
			3U
	);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	tx[0] = (calibration->offset_value >> 16) & 0xFFU;
	tx[1] = (calibration->offset_value >> 8) & 0xFFU;
	tx[2] = calibration->offset_value & 0xFFU;
	return AD4130_Write(
			adc_device_id,
			(uint8_t) (AD4130_OFFSET_0+setup),
			tx,
			3U
	);
}

static ErrorCode_t AD4130_Load_All_Setup_Calibrations(
		uint8_t *error_adc_device_id
)
{
	ErrorCode_t result;
	uint16_t config_value;

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		for (uint8_t setup = 0U; setup < 8U; setup++)
		{
			result = AD4130_Read_16_Bit(
					adc_device_id,
					(uint8_t) (AD4130_CONFIG_0+setup),
					&config_value
			);
			if (result == ERROR_CODE_NONE)
			{
				result = AD4130_Load_Setup_Calibration(
						adc_device_id,
						setup,
						(uint8_t) ((config_value >> 1) & 0x07U)
				);
			}
			if (result != ERROR_CODE_NONE)
			{
				*error_adc_device_id = adc_device_id;
				return result;
			}
		}
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t AD4130_Restore_Channels(
		const uint32_t channel_values
				[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT * 2U],
		uint8_t *error_adc_device_id
)
{
	ErrorCode_t result;
	uint8_t tx[3] = {0};

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		for (uint8_t i = 0U; i < AD4130_SENSOR_CHANNEL_COUNT * 2U; i++)
		{
			uint8_t channel = i;

			if (i >= AD4130_SENSOR_CHANNEL_COUNT)
			{
				channel = AD4130_REVERSE_CHANNEL_MIN
						+ i - AD4130_SENSOR_CHANNEL_COUNT;
			}

			tx[0] = (channel_values[adc_device_id-1U][i] >> 16) & 0xFFU;
			tx[1] = (channel_values[adc_device_id-1U][i] >> 8) & 0xFFU;
			tx[2] = channel_values[adc_device_id-1U][i] & 0xFFU;
			result = AD4130_Write(
					adc_device_id,
					(uint8_t) (AD4130_CHANNEL_0 + channel),
					tx,
					3U
			);
			if (result != ERROR_CODE_NONE)
			{
				*error_adc_device_id = adc_device_id;
				return result;
			}
		}
	}

	return ERROR_CODE_NONE;
}

ErrorCode_t AD4130_Internal_Calibrate(
		AD4130CalibrationResult_t results
				[AD4130_DEVICE_COUNT][AD4130_CALIBRATION_GAIN_COUNT],
		uint8_t *error_adc_device_id
)
{
	ErrorCode_t result;
	ErrorCode_t config_restore_result = ERROR_CODE_NONE;
	ErrorCode_t restore_result;
	uint32_t calibration_start_tick;
	uint32_t channel_values
			[AD4130_DEVICE_COUNT][AD4130_SENSOR_CHANNEL_COUNT * 2U] = {0};
	uint32_t offset_default = 0x800000U;
	uint16_t config_values[AD4130_DEVICE_COUNT] = {0};
	uint8_t calibration_setups[AD4130_DEVICE_COUNT] = {0};
	uint8_t restore_error_adc_device_id = 0U;
	uint8_t tx[3] = {0};

	if ((results == NULL) || (error_adc_device_id == NULL))
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}
	*error_adc_device_id = 0U;
	for (uint8_t adc_index = 0U; adc_index < AD4130_DEVICE_COUNT; adc_index++)
	{
		for (
				uint8_t gain_index = 0U;
				gain_index < AD4130_CALIBRATION_GAIN_COUNT;
				gain_index++
		)
		{
			results[adc_index][gain_index].gain = 0U;
			results[adc_index][gain_index].gain_value = 0U;
			results[adc_index][gain_index].offset_value = 0U;
			ad4130_calibration_cache[adc_index][gain_index].valid = 0U;
		}
	}

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		for (uint8_t i = 0U; i < AD4130_SENSOR_CHANNEL_COUNT * 2U; i++)
		{
			uint8_t channel = i;

			if (i >= AD4130_SENSOR_CHANNEL_COUNT)
			{
				channel = AD4130_REVERSE_CHANNEL_MIN
						+ i - AD4130_SENSOR_CHANNEL_COUNT;
			}

			result = AD4130_Read_24_Bit(
					adc_device_id,
					(uint8_t) (AD4130_CHANNEL_0 + channel),
					&channel_values[adc_device_id-1U][i]
			);
			if (result != ERROR_CODE_NONE)
			{
				*error_adc_device_id = adc_device_id;
				return result;
			}
		}

		if (
				(channel_values[adc_device_id-1U][0U]
				& 0x800000U) == 0U
		)
		{
			*error_adc_device_id = adc_device_id;
			return ERROR_CODE_AD4130_ILLEGAL_PARAM;
		}

		calibration_setups[adc_device_id-1U] =
				(channel_values[adc_device_id-1U][0U] >> 20) & 0x07U;
		result = AD4130_Read_16_Bit(
				adc_device_id,
				(uint8_t) (
						AD4130_CONFIG_0
						+ calibration_setups[adc_device_id-1U]
				),
				&config_values[adc_device_id-1U]
		);
		if (result != ERROR_CODE_NONE)
		{
			*error_adc_device_id = adc_device_id;
			return result;
		}
	}

	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		for (uint8_t i = 0U; i < AD4130_SENSOR_CHANNEL_COUNT * 2U; i++)
		{
			uint8_t channel = i;
			uint32_t channel_value = channel_values[adc_device_id-1U][i];

			if (i >= AD4130_SENSOR_CHANNEL_COUNT)
			{
				channel = AD4130_REVERSE_CHANNEL_MIN
						+ i - AD4130_SENSOR_CHANNEL_COUNT;
			}

			if (channel != 0U)
			{
				channel_value &= 0x7FFFFFU;
			}
			tx[0] = (channel_value >> 16) & 0xFFU;
			tx[1] = (channel_value >> 8) & 0xFFU;
			tx[2] = channel_value & 0xFFU;
			result = AD4130_Write(
					adc_device_id,
					(uint8_t) (AD4130_CHANNEL_0 + channel),
					tx,
					3U
			);
			if (result != ERROR_CODE_NONE)
			{
				*error_adc_device_id = adc_device_id;
				goto restore_channels;
			}
		}
	}

	for (
			uint8_t gain_index = 0U;
			gain_index < AD4130_CALIBRATION_GAIN_COUNT;
			gain_index++
	)
	{
		for (
				uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
				adc_device_id <= AD4130_DEVICE_ID_MAX;
				adc_device_id++
		)
		{
			uint16_t config_value = config_values[adc_device_id-1U];

			/* Bits 3-1: PGA_n */
			config_value &= 0b1111111111110001U;
			config_value |= (uint16_t)
					ad4130_calibration_gain_configs[gain_index] << 1;
			tx[0] = (config_value >> 8) & 0xFFU;
			tx[1] = config_value & 0xFFU;
			result = AD4130_Write(
					adc_device_id,
					(uint8_t) (
							AD4130_CONFIG_0
							+ calibration_setups[adc_device_id-1U]
					),
					tx,
					2U
			);
			if (result != ERROR_CODE_NONE)
			{
				*error_adc_device_id = adc_device_id;
				goto restore_configs;
			}

			tx[0] = (offset_default >> 16) & 0xFFU;
			tx[1] = (offset_default >> 8) & 0xFFU;
			tx[2] = offset_default & 0xFFU;
			result = AD4130_Write(
					adc_device_id,
					(uint8_t) (
							AD4130_OFFSET_0
							+ calibration_setups[adc_device_id-1U]
					),
					tx,
					3U
			);
			if (result != ERROR_CODE_NONE)
			{
				*error_adc_device_id = adc_device_id;
				goto restore_configs;
			}
		}

		/* start internal gain calibration */
		calibration_start_tick = HAL_GetTick();
		for (
				uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
				adc_device_id <= AD4130_DEVICE_ID_MAX;
				adc_device_id++
		)
		{
			result = AD4130_Start_Calibration(adc_device_id, 0b0110U);
			if (result != ERROR_CODE_NONE)
			{
				*error_adc_device_id = adc_device_id;
				goto restore_configs;
			}
		}
		result = AD4130_Wait_Calibrations(
				calibration_start_tick,
				AD4130_GAIN_CALIBRATION_TIMEOUT_MS,
				ERROR_CODE_AD4130_INTERNAL_GAIN_CALIBRATION_TIMEOUT,
				error_adc_device_id
		);
		if (result != ERROR_CODE_NONE)
		{
			goto restore_configs;
		}

		/* start internal offset calibration */
		calibration_start_tick = HAL_GetTick();
		for (
				uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
				adc_device_id <= AD4130_DEVICE_ID_MAX;
				adc_device_id++
		)
		{
			result = AD4130_Start_Calibration(adc_device_id, 0b0101U);
			if (result != ERROR_CODE_NONE)
			{
				*error_adc_device_id = adc_device_id;
				goto restore_configs;
			}
		}
		result = AD4130_Wait_Calibrations(
				calibration_start_tick,
				AD4130_OFFSET_CALIBRATION_TIMEOUT_MS,
				ERROR_CODE_AD4130_INTERNAL_OFFSET_CALIBRATION_TIMEOUT,
				error_adc_device_id
		);
		if (result != ERROR_CODE_NONE)
		{
			goto restore_configs;
		}

		for (
				uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
				adc_device_id <= AD4130_DEVICE_ID_MAX;
				adc_device_id++
		)
		{
			AD4130CalibrationResult_t *calibration_result =
					&results[adc_device_id-1U][gain_index];
			AD4130CalibrationCache_t *calibration_cache =
					&ad4130_calibration_cache
							[adc_device_id-1U][gain_index];

			calibration_result->gain =
					(uint16_t) (1U << (gain_index+3U));
			result = AD4130_Read_24_Bit(
					adc_device_id,
					(uint8_t) (
							AD4130_GAIN_0
							+ calibration_setups[adc_device_id-1U]
					),
					&calibration_result->gain_value
			);
			if (result == ERROR_CODE_NONE)
			{
				result = AD4130_Read_24_Bit(
						adc_device_id,
						(uint8_t) (
								AD4130_OFFSET_0
								+ calibration_setups[adc_device_id-1U]
						),
						&calibration_result->offset_value
				);
			}
			if (result != ERROR_CODE_NONE)
			{
				*error_adc_device_id = adc_device_id;
				goto restore_configs;
			}

			calibration_cache->gain_value =
					calibration_result->gain_value;
			calibration_cache->offset_value =
					calibration_result->offset_value;
			calibration_cache->valid = 1U;
		}
	}

restore_configs:
	for (
			uint8_t adc_device_id = AD4130_DEVICE_ID_MIN;
			adc_device_id <= AD4130_DEVICE_ID_MAX;
			adc_device_id++
	)
	{
		tx[0] = (config_values[adc_device_id-1U] >> 8) & 0xFFU;
		tx[1] = config_values[adc_device_id-1U] & 0xFFU;
		config_restore_result = AD4130_Write(
				adc_device_id,
				(uint8_t) (
						AD4130_CONFIG_0
						+ calibration_setups[adc_device_id-1U]
				),
				tx,
				2U
		);
		if (config_restore_result != ERROR_CODE_NONE)
		{
			restore_error_adc_device_id = adc_device_id;
			break;
		}
	}

restore_channels:
	restore_result = AD4130_Restore_Channels(
			channel_values,
			&restore_error_adc_device_id
	);
	if (
			(result == ERROR_CODE_NONE)
			&& (config_restore_result != ERROR_CODE_NONE)
	)
	{
		result = config_restore_result;
		*error_adc_device_id = restore_error_adc_device_id;
	}
	else if ((result == ERROR_CODE_NONE) && (restore_result != ERROR_CODE_NONE))
	{
		result = restore_result;
		*error_adc_device_id = restore_error_adc_device_id;
	}
	if (result == ERROR_CODE_NONE)
	{
		result = AD4130_Load_All_Setup_Calibrations(error_adc_device_id);
	}

	return result;
}

/* ------------------------------------------------------------------------ */

ErrorCode_t AD4130_Init(
		uint8_t adc_device_id,
		AD4130InitResult_t *init_result
)
{
	ErrorCode_t result;
	uint16_t control_val;
	uint16_t io_control_val;
	uint16_t misc_val;
	uint16_t error_en_val;
	uint8_t tx[2] = {0};

	if (init_result == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}
	init_result->id = 0U;
	init_result->status = 0U;
	init_result->error = 0U;

	result = AD4130_Reset(adc_device_id);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	/* Bits 13,10,9,8,5-2 */
	/* INT_REF_VAL,DATA_STATUS,CSB_EN,INT_REF_EN,MODE */
	control_val = 0b0000011100010000;

	tx[0] = (control_val >> 8) & 0xFFU;
	tx[1] = control_val & 0xFFU;
	result = AD4130_Write(adc_device_id, AD4130_ADC_CONTROL, tx, 2U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	/* Bits 3-0 */
	/* GPO_CTRL_P4-P1 */
	io_control_val = 0b0000000000001111;

	tx[0] = (io_control_val >> 8) & 0xFFU;
	tx[1] = io_control_val & 0xFFU;
	result = AD4130_Write(adc_device_id, AD4130_IO_CONTROL, tx, 2U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	/* Bit 13: CAL_RANGE_X2 */
	misc_val = 0b0010000000000000;

	tx[0] = (misc_val >> 8) & 0xFFU;
	tx[1] = misc_val & 0xFFU;
	result = AD4130_Write(adc_device_id, AD4130_MISC, tx, 2U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	/* Bits 6,5,4,3,1,0 */
	/* SPI_IGNORE_ERR_EN,SPI_SCLK_CNT_ERR_EN,SPI_READ_ERR_EN,SPI_WRITE_ERR_EN, */
	/* MM_CRC_ERR_EN,ROM_CRC_ERR_EN */
	error_en_val = 0b0000000001111011;

	tx[0] = (error_en_val >> 8) & 0xFFU;
	tx[1] = error_en_val & 0xFFU;
	result = AD4130_Write(adc_device_id, AD4130_ERROR_EN, tx, 2U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Set_Config(adc_device_id);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Set_Filter(
			adc_device_id,
			AD4130_FILTER_POST_FILTER_3
	);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Read_8_Bit(adc_device_id, AD4130_ID, &init_result->id);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}
	result = AD4130_Read_8_Bit(adc_device_id, AD4130_STATUS, &init_result->status);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}
	result = AD4130_Read_16_Bit(adc_device_id, AD4130_ERROR, &init_result->error);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Decode_Error(init_result->error);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	return AD4130_Check_Status_Error(adc_device_id, init_result->status);
}

static ErrorCode_t AD4130_Set_Config(uint8_t adc_device_id)
{
	ErrorCode_t result;
	uint16_t config_val;
	uint8_t tx[2] = {0};

	/* Bits 12-10,7,6,5-4,3-1 */
	/* I_OUT0_n,REF_BUFP_n,REF_BUFM_n,REF_SEL_n,PGA_n */
	config_val = 0b0000000011100110;

	for (uint8_t i = 0; i < 8U; i++)
	{
		tx[0] = (config_val >> 8) & 0xFFU;
		tx[1] = config_val & 0xFFU;
		result = AD4130_Write(adc_device_id, (uint8_t) (AD4130_CONFIG_0+i), tx, 2U);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
	}
	for (uint8_t channel = 0U; channel < AD4130_SENSOR_CHANNEL_COUNT; channel++)
	{
		ad4130_ranges[adc_device_id-1U][channel] =
				AD4130_RANGE_10UA_GAIN_8;
	}

	return ERROR_CODE_NONE;
}

ErrorCode_t AD4130_Set_Filter(uint8_t adc_device_id, uint8_t filter)
{
	ErrorCode_t result;
	uint32_t filter_value;
	uint8_t tx[3] = {0};

	/* Bits 23-21,15-12,10-0 */
	/* SETTLE_n,FILTER_MODE_n,FS_n */
	switch (filter)
	{
		case AD4130_FILTER_SINC3_REJ60:
			/* Sinc3 + REJ60 (FS 48) */
			filter_value = 0b110000000011000000110000;
			break;

		case AD4130_FILTER_POST_FILTER_3:
			/* Sinc3 + Post Filter 3 */
			filter_value = 0b110000000111000000110000;
			break;

		default:
			return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	tx[0] = (filter_value >> 16) & 0xFFU;
	tx[1] = (filter_value >> 8) & 0xFFU;
	tx[2] = filter_value & 0xFFU;

	for (uint8_t i = 0; i < 8U; i++)
	{
		result = AD4130_Write(adc_device_id, (uint8_t) (AD4130_FILTER_0+i), tx, 3U);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
	}

	return ERROR_CODE_NONE;
}

/* ------------------------------------------------------------------------ */

ErrorCode_t AD4130_Channel_0(uint8_t adc_device_id, AD4130Range_t range)
{
	ErrorCode_t result;
	uint32_t channel_0_val;
	uint8_t tx[3] = {0};

	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_AD4130_ILLEGAL_RANGE;
	}

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_0,SETUP_0,AINP_0(AIN0),AINM_0(AIN1),I_OUT0_CH_0(AIN6) */
	channel_0_val = 0b100000000000000100000110;
//	channel_0_val = 0b100000000010001000000000;

	tx[0] = (channel_0_val >> 16) & 0xFFU;
	tx[1] = (channel_0_val >> 8) & 0xFFU;
	tx[2] = channel_0_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_0, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	return AD4130_Set_Range(adc_device_id, 0U, range);
}

ErrorCode_t AD4130_Channel_1(uint8_t adc_device_id, AD4130Range_t range)
{
	ErrorCode_t result;
	uint32_t channel_1_val;
	uint8_t tx[3] = {0};

	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_AD4130_ILLEGAL_RANGE;
	}

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_1,SETUP_1,AINP_1(AIN8),AINM_1(AIN9),I_OUT0_CH_1(AIN7) */
	channel_1_val = 0b100100010000100100000111;
//	channel_1_val = 0b100100001010011000000100;

	tx[0] = (channel_1_val >> 16) & 0xFFU;
	tx[1] = (channel_1_val >> 8) & 0xFFU;
	tx[2] = channel_1_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_1, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	return AD4130_Set_Range(adc_device_id, 1U, range);
}

ErrorCode_t AD4130_Channel_2(uint8_t adc_device_id, AD4130Range_t range)
{
	ErrorCode_t result;
	uint32_t channel_2_val;
	uint8_t tx[3] = {0};

	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_AD4130_ILLEGAL_RANGE;
	}

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_2,SETUP_2,AINP_2(AIN11),AINM_2(AIN12),I_OUT0_CH_2(AIN10) */
	channel_2_val = 0b101000010110110000001010;
//	channel_2_val = 0b101000010010101000001000;

	tx[0] = (channel_2_val >> 16) & 0xFFU;
	tx[1] = (channel_2_val >> 8) & 0xFFU;
	tx[2] = channel_2_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_2, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	return AD4130_Set_Range(adc_device_id, 2U, range);
}

ErrorCode_t AD4130_Channel_3(uint8_t adc_device_id, AD4130Range_t range)
{
	ErrorCode_t result;
	uint32_t channel_3_val;
	uint8_t tx[3] = {0};

	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_AD4130_ILLEGAL_RANGE;
	}

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_3,SETUP_3,AINP_3(AIN14),AINM_3(AIN15),I_OUT0_CH_3(AIN13) */
	channel_3_val = 0b101100011100111100001101;
//	channel_3_val = 0b101100011010111000001100;

	tx[0] = (channel_3_val >> 16) & 0xFFU;
	tx[1] = (channel_3_val >> 8) & 0xFFU;
	tx[2] = channel_3_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_3, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	return AD4130_Set_Range(adc_device_id, 3U, range);
}

ErrorCode_t AD4130_Channel_4(uint8_t adc_device_id, AD4130Range_t range)
{
	ErrorCode_t result;
	uint32_t channel_4_val;
	uint8_t tx[3] = {0};

	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_AD4130_ILLEGAL_RANGE;
	}

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_4,SETUP_0,AINP_4(AIN1),AINM_4(AIN0),I_OUT0_CH_4(AIN6) */
	channel_4_val = 0b100000000010000000000110;
//	channel_4_val = 0b100000000000000100000110;

	tx[0] = (channel_4_val >> 16) & 0xFFU;
	tx[1] = (channel_4_val >> 8) & 0xFFU;
	tx[2] = channel_4_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_4, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	return AD4130_Set_Range(adc_device_id, 0U, range);
}

ErrorCode_t AD4130_Channel_5(uint8_t adc_device_id, AD4130Range_t range)
{
	ErrorCode_t result;
	uint32_t channel_5_val;
	uint8_t tx[3] = {0};

	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_AD4130_ILLEGAL_RANGE;
	}

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_5,SETUP_1,AINP_5(AIN9),AINM_5(AIN8),I_OUT0_CH_5(AIN7) */
	channel_5_val = 0b100100010010100000000111;
//	channel_5_val = 0b100100010000100100000111;

	tx[0] = (channel_5_val >> 16) & 0xFFU;
	tx[1] = (channel_5_val >> 8) & 0xFFU;
	tx[2] = channel_5_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_5, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	return AD4130_Set_Range(adc_device_id, 1U, range);
}

ErrorCode_t AD4130_Channel_6(uint8_t adc_device_id, AD4130Range_t range)
{
	ErrorCode_t result;
	uint32_t channel_6_val;
	uint8_t tx[3] = {0};

	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_AD4130_ILLEGAL_RANGE;
	}

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_6,SETUP_2,AINP_6(AIN12),AINM_6(AIN11),I_OUT0_CH_6(AIN10) */
	channel_6_val = 0b101000011000101100001010;
//	channel_6_val = 0b101000010110110000001010;

	tx[0] = (channel_6_val >> 16) & 0xFFU;
	tx[1] = (channel_6_val >> 8) & 0xFFU;
	tx[2] = channel_6_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_6, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	return AD4130_Set_Range(adc_device_id, 2U, range);
}

ErrorCode_t AD4130_Channel_7(uint8_t adc_device_id, AD4130Range_t range)
{
	ErrorCode_t result;
	uint32_t channel_7_val;
	uint8_t tx[3] = {0};

	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_AD4130_ILLEGAL_RANGE;
	}

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_7,SETUP_3,AINP_7(AIN15),AINM_7(AIN14),I_OUT0_CH_7(AIN13) */
	channel_7_val = 0b101100011110111000001101;
//	channel_7_val = 0b101100011100111100001101;

	tx[0] = (channel_7_val >> 16) & 0xFFU;
	tx[1] = (channel_7_val >> 8) & 0xFFU;
	tx[2] = channel_7_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_7, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	return AD4130_Set_Range(adc_device_id, 3U, range);
}

ErrorCode_t AD4130_Channel_Disable(
		uint8_t adc_device_id,
		uint8_t channel
)
{
	ErrorCode_t result;
	uint32_t channel_value;
	uint8_t tx[3] = {0};

	if (
			(channel > AD4130_REVERSE_CHANNEL_MAX)
			|| (
					(channel > AD4130_CHANNEL_MAX)
					&& (channel < AD4130_REVERSE_CHANNEL_MIN)
			)
	)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	result = AD4130_Read_24_Bit(
			adc_device_id,
			(uint8_t) (AD4130_CHANNEL_0+channel),
			&channel_value
	);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	/* Bit 23: ENABLE_n */
	channel_value &= 0b011111111111111111111111;
	tx[0] = (channel_value >> 16) & 0xFFU;
	tx[1] = (channel_value >> 8) & 0xFFU;
	tx[2] = channel_value & 0xFFU;

	return AD4130_Write(
			adc_device_id,
			(uint8_t) (AD4130_CHANNEL_0+channel),
			tx,
			3U
	);
}

ErrorCode_t AD4130_Reset_Measurement_Channels(uint8_t adc_device_id)
{
	ErrorCode_t result;
	AD4130Range_t range;

	/* preserve the CHANNEL_0 range */
	result = AD4130_Get_Range(adc_device_id, AD4130_CHANNEL_MIN, &range);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	/* disable all forward measurement channels */
	for (
			uint8_t channel = AD4130_CHANNEL_MIN;
			channel <= AD4130_CHANNEL_MAX;
			channel++
	)
	{
		result = AD4130_Channel_Disable(adc_device_id, channel);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
	}

	/* disable all reverse measurement channels */
	for (
			uint8_t channel = AD4130_REVERSE_CHANNEL_MIN;
			channel <= AD4130_REVERSE_CHANNEL_MAX;
			channel++
	)
	{
		result = AD4130_Channel_Disable(adc_device_id, channel);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
	}

	/* restore all current directions */
	for (
			uint8_t channel = AD4130_CHANNEL_MIN;
			channel <= AD4130_CHANNEL_MAX;
			channel++
	)
	{
		result = AD4130_Set_Current_Direction(adc_device_id, channel);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
	}

	/* enable CHANNEL_0 */
	return AD4130_Channel_0(adc_device_id, range);
}

ErrorCode_t AD4130_Get_Range(
		uint8_t adc_device_id,
		uint8_t channel,
		AD4130Range_t *range
)
{
	if (
			(adc_device_id < AD4130_DEVICE_ID_MIN)
			|| (adc_device_id > AD4130_DEVICE_ID_MAX)
	)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}
	if ((channel > AD4130_CHANNEL_MAX) || (range == NULL))
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	*range = ad4130_ranges[adc_device_id-1U][channel];

	return ERROR_CODE_NONE;
}

ErrorCode_t AD4130_Get_Range_Config(
		AD4130Range_t range,
		AD4130RangeConfig_t *config
)
{
	if (config == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}
	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_AD4130_ILLEGAL_RANGE;
	}

	*config = ad4130_range_configs[range].measurement;

	return ERROR_CODE_NONE;
}

ErrorCode_t AD4130_Set_Range(
		uint8_t adc_device_id,
		uint8_t channel,
		AD4130Range_t range
)
{
	ErrorCode_t result;
	ErrorCode_t range_result;
	uint32_t channel_value;
	uint16_t config_value;
	uint16_t original_config_value;
	uint8_t original_gain_config;
	uint8_t setup;
	uint8_t tx[2] = {0};

	if (
			(adc_device_id < AD4130_DEVICE_ID_MIN)
			|| (adc_device_id > AD4130_DEVICE_ID_MAX)
	)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}
	if (channel > AD4130_CHANNEL_MAX)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}
	if (range >= AD4130_RANGE_COUNT)
	{
		return ERROR_CODE_AD4130_ILLEGAL_RANGE;
	}

	result = AD4130_Read_24_Bit(
			adc_device_id,
			(uint8_t) (AD4130_CHANNEL_0+channel),
			&channel_value
	);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}
	setup = (channel_value >> 20) & 0x07U;

	result = AD4130_Read_16_Bit(
			adc_device_id,
			(uint8_t) (AD4130_CONFIG_0+setup),
			&config_value
	);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}
	original_config_value = config_value;
	original_gain_config = (config_value >> 1) & 0x07U;

	/* Bits 12-10,3-1: I_OUT0_n,PGA_n */
	config_value &= 0b1110001111110001U;
	config_value |= (
			(uint16_t) ad4130_range_configs[range].iout_config << 10
			| (uint16_t) ad4130_range_configs[range].gain_config << 1
	);

	tx[0] = (config_value >> 8) & 0xFFU;
	tx[1] = config_value & 0xFFU;

	result = AD4130_Write(
			adc_device_id,
			(uint8_t) (AD4130_CONFIG_0+setup),
			tx,
			2U
	);
	if (result == ERROR_CODE_NONE)
	{
		result = AD4130_Load_Setup_Calibration(
				adc_device_id,
				setup,
				ad4130_range_configs[range].gain_config
		);
	}
	if (result != ERROR_CODE_NONE)
	{
		range_result = result;
		tx[0] = (original_config_value >> 8) & 0xFFU;
		tx[1] = original_config_value & 0xFFU;
		result = AD4130_Write(
				adc_device_id,
				(uint8_t) (AD4130_CONFIG_0+setup),
				tx,
				2U
		);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}

		result = AD4130_Load_Setup_Calibration(
				adc_device_id,
				setup,
				original_gain_config
		);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}

		return range_result;
	}

	ad4130_ranges[adc_device_id-1U][channel] = range;

	return ERROR_CODE_NONE;
}

/* ------------------------------------------------------------------------ */

ErrorCode_t AD4130_FIFO_Enable(uint8_t adc_device_id, uint8_t watermark)
{
	uint32_t fifo_control_value;
	uint8_t tx[3] = {0};

	if ((watermark == 0U) || (watermark > 4U))
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	/* Bits 17-16 */
	/* FIFO_MODE */
	fifo_control_value = 0b000001010000001000000000 | watermark;

	tx[0] = (fifo_control_value >> 16) & 0xFFU;
	tx[1] = (fifo_control_value >> 8) & 0xFFU;
	tx[2] = fifo_control_value & 0xFFU;

	return AD4130_Write(adc_device_id, AD4130_FIFO_CONTROL, tx, 3U);
}

ErrorCode_t AD4130_FIFO_Disable(uint8_t adc_device_id)
{
	uint32_t fifo_control_value;
	uint8_t tx[3] = {0};

	/* Bits 17-16 */
	/* FIFO_MODE */
	fifo_control_value = 0b000001000000001000000000;

	tx[0] = (fifo_control_value >> 16) & 0xFFU;
	tx[1] = (fifo_control_value >> 8) & 0xFFU;
	tx[2] = fifo_control_value & 0xFFU;

	return AD4130_Write(adc_device_id, AD4130_FIFO_CONTROL, tx, 3U);
}

ErrorCode_t AD4130_FIFO_Ready(uint8_t adc_device_id, uint8_t *ready)
{
	ErrorCode_t result;
	uint8_t fifo_status;
	uint8_t status;

	if (ready == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}
	*ready = 0U;

	result = AD4130_Read_8_Bit(adc_device_id, AD4130_FIFO_STATUS, &fifo_status);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Decode_FIFO_Status(adc_device_id, fifo_status);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Read_8_Bit(adc_device_id, AD4130_STATUS, &status);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Check_Status_Error(adc_device_id, status);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	*ready = ((fifo_status & 0x02U) != 0U) ? 1U : 0U;

	return ERROR_CODE_NONE;
}

ErrorCode_t AD4130_FIFO_Read(
		uint8_t adc_device_id,
		AD4130FIFOSample_t *samples,
		uint8_t sample_count
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[2U + (4U * 4U)] = {0};
	uint8_t rx[2U + (4U * 4U)] = {0};
	uint16_t transfer_length;
	uint16_t sample_position;

	if (samples == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}
	if ((sample_count == 0U) || (sample_count > 4U))
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	device = AD4130_Get_Device(adc_device_id);
	if (device == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_DEVICE_ID;
	}

	tx[0] = 0x40U | AD4130_FIFO_DATA;  /* COMMS read, FIFO_DATA */
	tx[1] = sample_count;
	transfer_length = 2U + (4U * sample_count);

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(
			device->hspi,
			tx,
			rx,
			transfer_length,
			HAL_MAX_DELAY
	);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status != HAL_OK)
	{
		return AD4130_Convert_HAL_Status(status);
	}

	for (uint8_t i = 0U; i < sample_count; i++)
	{
		sample_position = 2U + (4U * i);
		samples[i].header = rx[sample_position];
		samples[i].data = (
				((uint32_t) rx[sample_position+1U] << 16)
				| ((uint32_t) rx[sample_position+2U] << 8)
				| rx[sample_position+3U]
		);
		if (
				(((samples[i].header & 0x20U) != 0U) && (i < sample_count-1U))
				|| (((samples[i].header & 0x20U) == 0U) && (i == sample_count-1U))
		)
		{
			return ERROR_CODE_AD4130_FIFO_READ;
		}
	}

	return ERROR_CODE_NONE;
}

ErrorCode_t AD4130_Set_Conversion_Mode(uint8_t adc_device_id, uint8_t mode)
{
	ErrorCode_t result;
	uint16_t control_value;
	uint8_t tx[2] = {0};

	if (
			(mode != AD4130_CONVERSION_MODE_NORMAL)
			&& (mode != AD4130_CONVERSION_MODE_STANDARD)
			&& (mode != AD4130_CONVERSION_MODE_FAST)
	)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	result = AD4130_Read_16_Bit(adc_device_id, AD4130_ADC_CONTROL, &control_value);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	/* Bits 5-2: MODE */
	control_value &= 0b1111111111000011U;
	control_value |= (uint16_t) mode << 2;

	tx[0] = (control_value >> 8) & 0xFFU;
	tx[1] = control_value & 0xFFU;

	return AD4130_Write(adc_device_id, AD4130_ADC_CONTROL, tx, 2U);
}

ErrorCode_t AD4130_Set_Current_Direction(
		uint8_t adc_device_id,
		uint8_t channel
)
{
	ErrorCode_t result;
	uint16_t io_control_value;
	uint16_t data_mask;
	uint8_t reversed;
	uint8_t control_pin;
	uint8_t tx[2] = {0};

	if (
			(channel > AD4130_REVERSE_CHANNEL_MAX)
			|| (
					(channel > AD4130_CHANNEL_MAX)
					&& (channel < AD4130_REVERSE_CHANNEL_MIN)
			)
	)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	reversed = (channel >= AD4130_REVERSE_CHANNEL_MIN) ? 1U : 0U;
	control_pin = (reversed != 0U)
			? channel - AD4130_REVERSE_CHANNEL_MIN
			: channel;

	result = AD4130_Read_16_Bit(
			adc_device_id,
			AD4130_IO_CONTROL,
			&io_control_value
	);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	data_mask = (uint16_t) (1U << (control_pin + 4U));

	/* Bits 7-4 */
	/* GPO_DATA_P4-P1 */
	if (reversed != 0U)
	{
		io_control_value |= data_mask;
	}
	else
	{
		io_control_value &= (uint16_t) ~data_mask;
	}

	tx[0] = (io_control_value >> 8) & 0xFFU;
	tx[1] = io_control_value & 0xFFU;

	return AD4130_Write(adc_device_id, AD4130_IO_CONTROL, tx, 2U);
}

void AD4130_Synchronize(void)
{
	HAL_GPIO_WritePin(AD4130_SYNC_GPIO_Port, AD4130_SYNC_Pin, GPIO_PIN_RESET);
	HAL_Delay(1U);
	HAL_GPIO_WritePin(AD4130_SYNC_GPIO_Port, AD4130_SYNC_Pin, GPIO_PIN_SET);
}

/* ------------------------------------------------------------------------ */
