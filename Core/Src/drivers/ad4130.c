#include "drivers/ad4130.h"

#include <stddef.h>

#include "gpio.h"
#include "spi.h"


/* ------------------------------------------------------------------------ */

typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
} AD4130Device_t;

static AD4130Device_t ad4130_devices[AD4130_DEVICE_COUNT] = {
	{
		.hspi = &hspi3,
		.cs_port = SPI3_CS1_GPIO_Port,
		.cs_pin = SPI3_CS1_Pin
	},
	{
		.hspi = &hspi3,
		.cs_port = SPI3_CS2_GPIO_Port,
		.cs_pin = SPI3_CS2_Pin
	}
};

static const float ad4130_iout_values[8] = {
	0.0f,
	100.0e-9f,
	10.0e-6f,
	20.0e-6f,
	50.0e-6f,
	100.0e-6f,
	150.0e-6f,
	200.0e-6f
};

static const uint8_t ad4130_iout_config_values[8] = {
	0b0000,  /* Off */
	0b0111,  /* 100 nA */
	0b0001,  /* 10 μA */
	0b0010,  /* 20 μA */
	0b0011,  /* 50 μA */
	0b0100,  /* 100 μA */
	0b0101,  /* 150 μA */
	0b0110   /* 200 μA */
};

AD4130Iouts_t ad4130_iouts[AD4130_DEVICE_COUNT] = {0};

static ErrorCode_t AD4130_Config(uint8_t adc_device_id);
static ErrorCode_t AD4130_Filter(uint8_t adc_device_id);
static ErrorCode_t AD4130_Set_IOUT_Level(
		uint8_t adc_device_id,
		uint8_t setup,
		uint8_t iout_level
);
static ErrorCode_t AD4130_Run_Calibration(
		uint8_t adc_device_id,
		uint8_t mode
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

	if ((adc_device_id < AD4130_DEVICE_ID_MIN) || (adc_device_id > AD4130_DEVICE_ID_MAX))
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
	if ((adc_device_id < AD4130_DEVICE_ID_MIN) || (adc_device_id > AD4130_DEVICE_ID_MAX))
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

static ErrorCode_t AD4130_Run_Calibration(
		uint8_t adc_device_id,
		uint8_t mode
)
{
	ErrorCode_t result;
	uint16_t control_value;
	uint8_t status;
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
	result = AD4130_Write(adc_device_id, AD4130_ADC_CONTROL, tx, 2U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	while (1)
	{
		result = AD4130_Read_8_Bit(adc_device_id, AD4130_STATUS, &status);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
		if ((status & 0x80U) == 0U)
		{
			return AD4130_Check_Status_Error(adc_device_id, status);
		}
		HAL_Delay(1U);
	}
}

ErrorCode_t AD4130_Internal_Calibrate(
		uint8_t adc_device_id,
		uint8_t channel,
		uint8_t *setup_pointer,
		uint32_t *gain_value,
		uint32_t *offset_value
)
{
	ErrorCode_t result;
	ErrorCode_t restore_result;
	uint32_t channel_values[AD4130_SENSOR_CHANNEL_COUNT] = {0};
	uint32_t calibrated_gain = 0U;
	uint32_t calibrated_offset = 0U;
	uint32_t offset_default = 0x800000U;
	uint8_t setup;
	uint8_t tx[3] = {0};

	if (
			(channel >= AD4130_SENSOR_CHANNEL_COUNT)
			|| (setup_pointer == NULL)
			|| (gain_value == NULL)
			|| (offset_value == NULL)
	)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}
	*setup_pointer = 0xFFU;
	*gain_value = 0U;
	*offset_value = 0U;

	for (uint8_t i = 0U; i < AD4130_SENSOR_CHANNEL_COUNT; i++)
	{
		result = AD4130_Read_24_Bit(
				adc_device_id,
				(uint8_t) (AD4130_CHANNEL_0+i),
				&channel_values[i]
		);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
	}

	if ((channel_values[channel] & 0x800000U) == 0U)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}

	for (uint8_t i = 0U; i < AD4130_SENSOR_CHANNEL_COUNT; i++)
	{
		uint32_t channel_value = channel_values[i];

		if (i != channel)
		{
			channel_value &= 0x7FFFFFU;
		}
		tx[0] = (channel_value >> 16) & 0xFFU;
		tx[1] = (channel_value >> 8) & 0xFFU;
		tx[2] = channel_value & 0xFFU;
		result = AD4130_Write(
				adc_device_id,
				(uint8_t) (AD4130_CHANNEL_0+i),
				tx,
				3U
		);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
	}

	setup = (channel_values[channel] >> 20) & 0x07U;
	tx[0] = (offset_default >> 16) & 0xFFU;
	tx[1] = (offset_default >> 8) & 0xFFU;
	tx[2] = offset_default & 0xFFU;
	result = AD4130_Write(
			adc_device_id,
			(uint8_t) (AD4130_OFFSET_0+setup),
			tx,
			3U
	);
	if (result == ERROR_CODE_NONE)
	{
		result = AD4130_Run_Calibration(adc_device_id, 0b0110U);
	}
	if (result == ERROR_CODE_NONE)
	{
		result = AD4130_Run_Calibration(adc_device_id, 0b0101U);
	}
	if (result == ERROR_CODE_NONE)
	{
		result = AD4130_Read_24_Bit(
				adc_device_id,
				(uint8_t) (AD4130_GAIN_0+setup),
				&calibrated_gain
		);
	}
	if (result == ERROR_CODE_NONE)
	{
		result = AD4130_Read_24_Bit(
				adc_device_id,
				(uint8_t) (AD4130_OFFSET_0+setup),
				&calibrated_offset
		);
	}

	for (uint8_t i = 0U; i < AD4130_SENSOR_CHANNEL_COUNT; i++)
	{
		tx[0] = (channel_values[i] >> 16) & 0xFFU;
		tx[1] = (channel_values[i] >> 8) & 0xFFU;
		tx[2] = channel_values[i] & 0xFFU;
		restore_result = AD4130_Write(
				adc_device_id,
				(uint8_t) (AD4130_CHANNEL_0+i),
				tx,
				3U
		);
		if ((result == ERROR_CODE_NONE) && (restore_result != ERROR_CODE_NONE))
		{
			result = restore_result;
		}
	}

	if (result == ERROR_CODE_NONE)
	{
		*setup_pointer = setup;
		*gain_value = calibrated_gain;
		*offset_value = calibrated_offset;
	}

	return result;
}

void AD4130_Synchronize(void)
{
	HAL_GPIO_WritePin(AD4130_SYNC_GPIO_Port, AD4130_SYNC_Pin, GPIO_PIN_RESET);
	HAL_Delay(1U);
	HAL_GPIO_WritePin(AD4130_SYNC_GPIO_Port, AD4130_SYNC_Pin, GPIO_PIN_SET);
}

ErrorCode_t AD4130_Set_Conversion_Mode(uint8_t adc_device_id, uint8_t mode)
{
	ErrorCode_t result;
	uint16_t control_value;
	uint8_t tx[2] = {0};

	if (
			(mode != AD4130_CONVERSION_MODE_NORMAL)
			&& (mode != AD4130_CONVERSION_MODE_SYNC)
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

ErrorCode_t AD4130_FIFO_Enable(uint8_t adc_device_id, uint8_t watermark)
{
	uint32_t fifo_control_value;
	uint8_t tx[3] = {0};

	if ((watermark == 0U) || (watermark > AD4130_SENSOR_CHANNEL_COUNT))
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
	uint8_t tx[2U + (4U * AD4130_SENSOR_CHANNEL_COUNT)] = {0};
	uint8_t rx[2U + (4U * AD4130_SENSOR_CHANNEL_COUNT)] = {0};
	uint16_t transfer_length;
	uint16_t sample_position;

	if (samples == NULL)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}
	if ((sample_count == 0U) || (sample_count > AD4130_SENSOR_CHANNEL_COUNT))
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

/* ------------------------------------------------------------------------ */

ErrorCode_t AD4130_Init(
		uint8_t adc_device_id,
		AD4130InitResult_t *init_result
)
{
	ErrorCode_t result;
	uint16_t control_val;
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

	/* Bits 13,10,9,8 */
	/* INT_REF_VAL,DATA_STATUS,CSB_EN,INT_REF_EN */
	control_val = 0b0010011100010000;

	tx[0] = (control_val >> 8) & 0xFFU;
	tx[1] = control_val & 0xFFU;
	result = AD4130_Write(adc_device_id, AD4130_ADC_CONTROL, tx, 2U);
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

	result = AD4130_Config(adc_device_id);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Filter(adc_device_id);
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

static ErrorCode_t AD4130_Config(uint8_t adc_device_id)
{
	ErrorCode_t result;
	uint16_t config_val_common;
	uint8_t tx[2] = {0};

	/* Bits 12-10,7,6,5-4,3-1 */
	/* I_OUT0_n,REF_BUFP_n,REF_BUFM_n,REF_SEL_n,PGA_n */
	config_val_common = 0b0000000011101110;

	for (uint8_t i = 0; i < 8U; i++)
	{
		tx[0] = (config_val_common >> 8) & 0xFFU;
		tx[1] = config_val_common & 0xFFU;
		result = AD4130_Write(adc_device_id, (uint8_t) (AD4130_CONFIG_0+i), tx, 2U);
		if (result != ERROR_CODE_NONE)
		{
			return result;
		}
	}

	return ERROR_CODE_NONE;
}

static ErrorCode_t AD4130_Set_IOUT_Level(
		uint8_t adc_device_id,
		uint8_t setup,
		uint8_t iout_level
)
{
	ErrorCode_t result;
	uint16_t config_value;
	uint8_t tx[2] = {0};

	if (setup > 7U)
	{
		return ERROR_CODE_AD4130_ILLEGAL_PARAM;
	}
	if (iout_level > 7U)
	{
		return ERROR_CODE_AD4130_ILLEGAL_IOUT;
	}

	result = AD4130_Read_16_Bit(
			adc_device_id,
			(uint8_t) (AD4130_CONFIG_0+setup),
			&config_value
	);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	/* Bits 12-10: I_OUT0_n */
	config_value &= 0b1110001111111111;
	config_value |= (uint16_t) ad4130_iout_config_values[iout_level] << 10;

	tx[0] = (config_value >> 8) & 0xFFU;
	tx[1] = config_value & 0xFFU;

	return AD4130_Write(
			adc_device_id,
			(uint8_t) (AD4130_CONFIG_0+setup),
			tx,
			2U
	);
}

static ErrorCode_t AD4130_Filter(uint8_t adc_device_id)
{
	ErrorCode_t result;
	uint32_t filter_val;
	uint8_t tx[3] = {0};

	/* Bits 23-21,15-12,10-0 */
	/* SETTLE_n,FILTER_MODE_n,FS_n */
	filter_val = 0b111000000011000000110000;

	tx[0] = (filter_val >> 16) & 0xFFU;
	tx[1] = (filter_val >> 8) & 0xFFU;
	tx[2] = filter_val & 0xFFU;

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

ErrorCode_t AD4130_Channel_0(uint8_t adc_device_id, uint8_t iout_level)
{
	if (iout_level > 7U)
	{
		return ERROR_CODE_AD4130_ILLEGAL_IOUT;
	}

	ErrorCode_t result;
	uint32_t channel_0_val;
	uint8_t tx[3] = {0};

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_0,SETUP_0,AINP_0(AIN0),AINM_0(AIN1),I_OUT0_CH_0(AIN6) */
	channel_0_val = 0b100000000000000100000110;

	tx[0] = (channel_0_val >> 16) & 0xFFU;
	tx[1] = (channel_0_val >> 8) & 0xFFU;
	tx[2] = channel_0_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_0, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Set_IOUT_Level(adc_device_id, 0U, iout_level);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	ad4130_iouts[adc_device_id - 1U].level_1 = iout_level;
	ad4130_iouts[adc_device_id - 1U].i_1 = ad4130_iout_values[iout_level];

	return result;
}

ErrorCode_t AD4130_Channel_1(uint8_t adc_device_id, uint8_t iout_level)
{
	if (iout_level > 7U)
	{
		return ERROR_CODE_AD4130_ILLEGAL_IOUT;
	}

	ErrorCode_t result;
	uint32_t channel_1_val;
	uint8_t tx[3] = {0};

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_1,SETUP_1,AINP_1(AIN8),AINM_1(AIN9),I_OUT0_CH_1(AIN7) */
	channel_1_val = 0b100100010000100100000111;

	tx[0] = (channel_1_val >> 16) & 0xFFU;
	tx[1] = (channel_1_val >> 8) & 0xFFU;
	tx[2] = channel_1_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_1, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Set_IOUT_Level(adc_device_id, 1U, iout_level);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	ad4130_iouts[adc_device_id - 1U].level_2 = iout_level;
	ad4130_iouts[adc_device_id - 1U].i_2 = ad4130_iout_values[iout_level];

	return result;
}

ErrorCode_t AD4130_Channel_2(uint8_t adc_device_id, uint8_t iout_level)
{
	if (iout_level > 7U)
	{
		return ERROR_CODE_AD4130_ILLEGAL_IOUT;
	}

	ErrorCode_t result;
	uint32_t channel_2_val;
	uint8_t tx[3] = {0};

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_2,SETUP_2,AINP_2(AIN11),AINM_2(AIN12),I_OUT0_CH_2(AIN10) */
	channel_2_val = 0b101000010110110000001010;

	tx[0] = (channel_2_val >> 16) & 0xFFU;
	tx[1] = (channel_2_val >> 8) & 0xFFU;
	tx[2] = channel_2_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_2, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Set_IOUT_Level(adc_device_id, 2U, iout_level);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	ad4130_iouts[adc_device_id - 1U].level_3 = iout_level;
	ad4130_iouts[adc_device_id - 1U].i_3 = ad4130_iout_values[iout_level];

	return result;
}

ErrorCode_t AD4130_Channel_3(uint8_t adc_device_id, uint8_t iout_level)
{
	if (iout_level > 7U)
	{
		return ERROR_CODE_AD4130_ILLEGAL_IOUT;
	}

	ErrorCode_t result;
	uint32_t channel_3_val;
	uint8_t tx[3] = {0};

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_3,SETUP_3,AINP_3(AIN14),AINM_3(AIN15),I_OUT0_CH_3(AIN13) */
	channel_3_val = 0b101100011100111100001101;

	tx[0] = (channel_3_val >> 16) & 0xFFU;
	tx[1] = (channel_3_val >> 8) & 0xFFU;
	tx[2] = channel_3_val & 0xFFU;

	result = AD4130_Write(adc_device_id, AD4130_CHANNEL_3, tx, 3U);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	result = AD4130_Set_IOUT_Level(adc_device_id, 3U, iout_level);
	if (result != ERROR_CODE_NONE)
	{
		return result;
	}

	ad4130_iouts[adc_device_id - 1U].level_4 = iout_level;
	ad4130_iouts[adc_device_id - 1U].i_4 = ad4130_iout_values[iout_level];

	return result;
}

/* ------------------------------------------------------------------------ */
