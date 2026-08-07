#include "drivers/ad4130.h"

/* ------------------------------------------------------------------------ */

typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
} AD4130Device_t;

static AD4130Device_t ad4130_devices[2] = {
	{
		.hspi = &hspi1,
		.cs_port = SPI1_CS1_GPIO_Port,
		.cs_pin = SPI1_CS1_Pin
	},
	{
		.hspi = &hspi1,
		.cs_port = SPI1_CS2_GPIO_Port,
		.cs_pin = SPI1_CS2_Pin
	}
};

const float ad4130_iout_values[8] = {
	0.0f,
	100.0e-9f,
	10.0e-6f,
	20.0e-6f,
	50.0e-6f,
	100.0e-6f,
	150.0e-6f,
	200.0e-6f
};

AD4130Iouts_t ad4130_iouts[2] = {0};

/* ------------------------------------------------------------------------ */

static AD4130Device_t *AD4130_Get_Device(uint8_t adc_device_id)
{
	if (adc_device_id < 1U || adc_device_id > 2U)
	{
		return NULL;  /* Invalid ADC device ID */
	}
	return &ad4130_devices[adc_device_id - 1U];
}

HAL_StatusTypeDef AD4130_Read_8_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint8_t *value
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[2] = {0};
	uint8_t rx[2] = {0};

	device = AD4130_Get_Device(adc_device_id);
	if ((device == NULL) || (value == NULL))
	{
		return HAL_ERROR;
	}

	tx[0] = 0x40U | reg_addr;  /* 0b01000000, COMMS read */

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(device->hspi, tx, rx, 2U, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		*value = rx[1];
	}
	return status;
}

HAL_StatusTypeDef AD4130_Read_16_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint16_t *value
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[3] = {0};
	uint8_t rx[3] = {0};

	device = AD4130_Get_Device(adc_device_id);
	if ((device == NULL) || (value == NULL))
	{
		return HAL_ERROR;
	}

	tx[0] = 0x40U | reg_addr;  /* 0b01000000, COMMS read */

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(device->hspi, tx, rx, 3U, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		*value = (
			((uint16_t) rx[1] << 8)
			| rx[2]
		);
	}
	return status;
}

HAL_StatusTypeDef AD4130_Read_24_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint32_t *value
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[4] = {0};
	uint8_t rx[4] = {0};

	device = AD4130_Get_Device(adc_device_id);
	if ((device == NULL) || (value == NULL))
	{
		return HAL_ERROR;
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
	return status;
}

HAL_StatusTypeDef AD4130_Read_32_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint32_t *value
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[5] = {0};
	uint8_t rx[5] = {0};

	device = AD4130_Get_Device(adc_device_id);
	if ((device == NULL) || (value == NULL))
	{
		return HAL_ERROR;
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
	return status;
}

HAL_StatusTypeDef AD4130_Write(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		const uint8_t *data,
		uint16_t len
)
{
	AD4130Device_t *device;
	HAL_StatusTypeDef status;
	uint8_t tx[4] = {0};

	device = AD4130_Get_Device(adc_device_id);
	if ((device == NULL) || (data == NULL))
	{
		return HAL_ERROR;
	}

	if ((len == 0U) || (len > 3U))
	{
		return HAL_ERROR;  /* write length invalid */
	}

	tx[0] = reg_addr & 0x3FU;  /*  0b00RS[5:0], COMMS write*/
	for (uint16_t i = 0; i < len; i++)
	{
		tx[i + 1U] = data[i];
	}

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(device->hspi, tx, len+1U, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		HAL_Delay(1);
	}
	return status;
}

HAL_StatusTypeDef AD4130_Reset(uint8_t adc_device_id)
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
		return HAL_ERROR;
	}

	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(device->hspi, tx, 8U, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		HAL_Delay(10);
	}
	return status;
}

HAL_StatusTypeDef AD4130_Init(
		uint8_t adc_device_id,
		AD4130InitResult_t *init_result
)
{
	HAL_StatusTypeDef status;
	uint16_t control_val;
	uint16_t error_en_val;
	uint8_t tx[2] = {0};

	if (init_result == NULL)
	{
		return HAL_ERROR;
	}
	init_result->id = 0U;
	init_result->status = 0U;
	init_result->error = 0U;

	status = AD4130_Reset(adc_device_id);
	if (status != HAL_OK)
	{
		return status;
	}

	/* Bits 13,10,9,8 */
	/* INT_REF_VAL,DATA_STATUS,CSB_EN,INT_REF_EN */
	control_val = 0b0010011100000000;

	tx[0] = (control_val >> 8) & 0xFFU;
	tx[1] = control_val & 0xFFU;
	status = AD4130_Write(adc_device_id, AD4130_ADC_CONTROL, tx, 2U);
	if (status != HAL_OK)
	{
		return status;
	}

	/* Bits 6,5,4,3 */
	/* SPI_IGNORE_ERR_EN,SPI_SCLK_CNT_ERR_EN,SPI_READ_ERR_EN,SPI_WRITE_ERR_EN */
	error_en_val = 0b0000000001111000;

	tx[0] = (error_en_val >> 8) & 0xFFU;
	tx[1] = error_en_val & 0xFFU;
	status = AD4130_Write(adc_device_id, AD4130_ERROR_EN, tx, 2U);
	if (status != HAL_OK)
	{
		return status;
	}

	status = AD4130_Config(adc_device_id);
	if (status != HAL_OK)
	{
		return status;
	}

	status = AD4130_Filter(adc_device_id);
	if (status != HAL_OK)
	{
		return status;
	}

	status = AD4130_Read_8_Bit(adc_device_id, AD4130_ID, &init_result->id);
	if (status != HAL_OK)
	{
		return status;
	}
	status = AD4130_Read_8_Bit(adc_device_id, AD4130_STATUS, &init_result->status);
	if (status != HAL_OK)
	{
		return status;
	}
	status = AD4130_Read_16_Bit(adc_device_id, AD4130_ERROR, &init_result->error);
	if (status != HAL_OK)
	{
		return status;
	}

	return HAL_OK;
}

/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef AD4130_Config(uint8_t adc_device_id)
{
	HAL_StatusTypeDef status;
	uint16_t config_val;
	uint16_t config_val_common;
	uint8_t config_val_iout[8];
	uint8_t tx[2] = {0};

	/* Bits 12-10,7,6,5-4,3-1 */
	/* I_OUT0_n,REF_BUFP_n,REF_BUFM_n,REF_SEL_n,PGA_n */
	config_val_common = 0b0000000011101110;

	config_val_iout[0] = 0b0000;  /* Off */
	config_val_iout[1] = 0b0111;  /* 100 nA */
	config_val_iout[2] = 0b0001;  /* 10 μA */
	config_val_iout[3] = 0b0010;  /* 20 μA */
	config_val_iout[4] = 0b0011;  /* 50 μA */
	config_val_iout[5] = 0b0100;  /* 100 μA */
	config_val_iout[6] = 0b0101;  /* 150 μA */
	config_val_iout[7] = 0b0110;  /* 200 μA */

	for (uint8_t i = 0; i < 8U; i++)
	{
		config_val = config_val_common | ((uint16_t)config_val_iout[i] << 10);

		tx[0] = (config_val >> 8) & 0xFFU;
		tx[1] = config_val & 0xFFU;
		status = AD4130_Write(adc_device_id, (uint8_t)(AD4130_CONFIG_0+i), tx, 2U);
		if (status != HAL_OK)
		{
			return status;
		}
	}

	return HAL_OK;
}

HAL_StatusTypeDef AD4130_Filter(uint8_t adc_device_id)
{
	HAL_StatusTypeDef status;
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
		status = AD4130_Write(adc_device_id, (uint8_t)(AD4130_FILTER_0+i), tx, 3U);
		if (status != HAL_OK)
		{
			return status;
		}
	}

	return HAL_OK;
}

/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef AD4130_Channel_0(
		uint8_t adc_device_id,
		uint8_t iout_level
)
{
	if (iout_level > 7U)
	{
		return HAL_ERROR;  /* Invalid I_OUT level */
	}

	HAL_StatusTypeDef status;
	uint32_t channel_0_val;
	uint8_t tx[3] = {0};

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_0,SETUP_0,AINP_0,AINM_0,I_OUT0_CH_0 */
	channel_0_val = (
			0b100000000100001100000000
			| ((uint32_t)iout_level << 20)
	);

	tx[0] = (channel_0_val >> 16) & 0xFFU;
	tx[1] = (channel_0_val >> 8) & 0xFFU;
	tx[2] = channel_0_val & 0xFFU;

	status = AD4130_Write(adc_device_id, AD4130_CHANNEL_0, tx, 3U);
	if (status != HAL_OK)
	{
		return status;
	}

	ad4130_iouts[adc_device_id - 1U].level_1 = iout_level;
	ad4130_iouts[adc_device_id - 1U].i_1 = ad4130_iout_values[iout_level];

	return status;
}

/* ------------------------------------------------------------------------ */
