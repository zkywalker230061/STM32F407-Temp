#include "bsp/ad4130.h"

/* ADC1 (CS = PA4) */
/* ------------------------------------------------------------------------ */
void ADC1_Reset(void)
{
	uint8_t tx[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx, 8, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_SET);

	HAL_Delay(10);
}

void ADC1_Init(void)
{
	uint16_t control_val;
	uint16_t error_en_val;
	uint8_t tx[2];

	ADC1_Reset();

	/* Bit 13,10,9,8 */
	/* INT_REF_VAL,DATA_STATUS,CSB_EN,INT_REF_EN */
	control_val = 0b0010011100000000;
	tx[0] = (control_val >> 8) & 0xFF;
	tx[1] = control_val & 0xFF;
	ADC1_Write(AD4130_ADC_CONTROL, tx, 2);

	/* Bits 6,5,4,3 */
	/* SPI_IGNORE_ERR_EN,SPI_SCLK_CNT_ERR_EN,SPI_READ_ERR_EN,SPI_WRITE_ERR_EN */
	error_en_val = 0b0000000001111000;
	tx[0] = (error_en_val >> 8) & 0xFF;
	tx[1] = error_en_val & 0xFF;
	ADC1_Write(AD4130_ERROR_EN, tx, 2);
}

uint8_t ADC1_Read_8_bit(uint8_t reg_addr)
{
	uint8_t tx_cache[2];
	uint8_t rx_cache[2];
	uint8_t rx;

	tx_cache[0] = 0b01000000 | reg_addr;  /* COMMS, read */
	tx_cache[1] = 0b00000000;
	rx = 0;

	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_cache, rx_cache, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_SET);

	rx = rx_cache[1];
	return rx;
}

uint16_t ADC1_Read_16_bit(uint8_t reg_addr)
{
	uint8_t tx_cache[3];
	uint8_t rx_cache[3];
	uint16_t rx;

	tx_cache[0] = 0b01000000 | reg_addr;  /* COMMS, read */
	tx_cache[1] = 0b00000000;
	tx_cache[2] = 0b00000000;
	rx = 0;

	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_cache, rx_cache, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_SET);

	rx = (
			((uint16_t) rx_cache[1] << 8)
			| rx_cache[2]
	);
	return rx;
}

uint32_t ADC1_Read_24_bit(uint8_t reg_addr)
{
	uint8_t tx_cache[4];
	uint8_t rx_cache[4];
	uint32_t rx;

	tx_cache[0] = 0b01000000 | reg_addr;  /* COMMS, read */
	tx_cache[1] = 0b00000000;
	tx_cache[2] = 0b00000000;
	tx_cache[3] = 0b00000000;
	rx = 0;

	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_cache, rx_cache, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_SET);

	rx = (
			((uint32_t) rx_cache[1] << 16)
			| ((uint32_t) rx_cache[2] << 8)
			| rx_cache[3]
	);
	return rx;
}

uint32_t ADC1_Read_32_bit(uint8_t reg_addr)
{
	uint8_t tx_cache[5];
	uint8_t rx_cache[5];
	uint32_t rx;

	tx_cache[0] = 0b01000000 | reg_addr;  /* COMMS, read */
	tx_cache[1] = 0b00000000;
	tx_cache[2] = 0b00000000;
	tx_cache[3] = 0b00000000;
	tx_cache[4] = 0b00000000;
	rx = 0;

	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_cache, rx_cache, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_SET);

	rx = (
			((uint32_t) rx_cache[1] << 24)
			| ((uint32_t) rx_cache[2] << 16)
			| ((uint32_t) rx_cache[3] << 8)
			| rx_cache[4]
	);
	return rx;
}

void ADC1_Write(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t tx_cache[1 + len];

	tx_cache[0] = reg_addr;
	for (uint16_t i = 0; i < len; i++) {
		tx_cache[1 + i] = data[i];
	}

	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx_cache, 1 + len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_SET);

	HAL_Delay(1);
}

uint8_t ADC1_ReadID(void)
{
	return ADC1_Read_8_bit(AD4130_ID);
}

/* ADC2 (CS = PC4) */
/* ------------------------------------------------------------------------ */
void ADC2_Reset(void)
{
	uint8_t tx[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx, 8, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_SET);

	HAL_Delay(10);
}

void ADC2_Init(void) {
	uint16_t control_val;
	uint16_t error_en_val;
	uint8_t tx[2];

	ADC2_Reset();

	/* Bit 13,10,9,8 */
	/* INT_REF_VAL,DATA_STATUS,CSB_EN,INT_REF_EN */
	control_val = 0b0010011100000000;
	tx[0] = (control_val >> 8) & 0xFF;
	tx[1] = control_val & 0xFF;
	ADC2_Write(AD4130_ADC_CONTROL, tx, 2);

	/* Bits 6,5,4,3 */
	/* SPI_IGNORE_ERR_EN,SPI_SCLK_CNT_ERR_EN,SPI_READ_ERR_EN,SPI_WRITE_ERR_EN */
	error_en_val = 0b0000000001111000;
	tx[0] = (error_en_val >> 8) & 0xFF;
	tx[1] = error_en_val & 0xFF;
	ADC2_Write(AD4130_ERROR_EN, tx, 2);
}

uint8_t ADC2_Read_8_bit(uint8_t reg_addr)
{
	uint8_t tx_cache[2];
	uint8_t rx_cache[2];
	uint8_t rx;

	tx_cache[0] = 0b01000000 | reg_addr;  /* COMMS, read */
	tx_cache[1] = 0b00000000;
	rx = 0;

	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_cache, rx_cache, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_SET);

	rx = rx_cache[1];
	return rx;
}

uint16_t ADC2_Read_16_bit(uint8_t reg_addr)
{
	uint8_t tx_cache[3];
	uint8_t rx_cache[3];
	uint16_t rx;

	tx_cache[0] = 0b01000000 | reg_addr;  /* COMMS, read */
	tx_cache[1] = 0b00000000;
	tx_cache[2] = 0b00000000;
	rx = 0;

	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_cache, rx_cache, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_SET);

	rx = (
			((uint16_t) rx_cache[1] << 8)
			| rx_cache[2]
	);
	return rx;
}

uint32_t ADC2_Read_24_bit(uint8_t reg_addr)
{
	uint8_t tx_cache[4];
	uint8_t rx_cache[4];
	uint32_t rx;

	tx_cache[0] = 0b01000000 | reg_addr;  /* COMMS, read */
	tx_cache[1] = 0b00000000;
	tx_cache[2] = 0b00000000;
	tx_cache[3] = 0b00000000;
	rx = 0;

	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_cache, rx_cache, 4, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_SET);

	rx = (
			((uint32_t) rx_cache[1] << 16)
			| ((uint32_t) rx_cache[2] << 8)
			| rx_cache[3]
	);
	return rx;
}

uint32_t ADC2_Read_32_bit(uint8_t reg_addr)
{
	uint8_t tx_cache[5];
	uint8_t rx_cache[5];
	uint32_t rx;

	tx_cache[0] = 0b01000000 | reg_addr;  /* COMMS, read */
	tx_cache[1] = 0b00000000;
	tx_cache[2] = 0b00000000;
	tx_cache[3] = 0b00000000;
	tx_cache[4] = 0b00000000;
	rx = 0;

	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_cache, rx_cache, 5, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_SET);

	rx = (
			((uint32_t) rx_cache[1] << 24)
			| ((uint32_t) rx_cache[2] << 16)
			| ((uint32_t) rx_cache[3] << 8)
			| rx_cache[4]
	);
	return rx;
}

void ADC2_Write(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t tx_cache[1 + len];

	tx_cache[0] = reg_addr;
	for (uint16_t i = 0; i < len; i++) {
		tx_cache[1 + i] = data[i];
	}

	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx_cache, 1 + len, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_SET);

	HAL_Delay(1);
}

uint8_t ADC2_ReadID(void)
{
	return ADC2_Read_8_bit(AD4130_ID);
}

void ADC2_Config(void)
{
	uint16_t config_val;
	uint16_t config_val_common;
	uint8_t config_val_iout[8];
	uint8_t tx[2];

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
	for (uint8_t i = 0; i < 8; i++)
	{
		config_val = config_val_common | (config_val_iout[i] << 10);
		tx[0] = (config_val >> 8) & 0xFF;
		tx[1] = config_val & 0xFF;
		ADC2_Write(AD4130_CONFIG_0 + i, tx, 2);
	}
}

void ADC2_Filter(void)
{
	uint32_t filter_val;
	uint8_t tx[3];

	/* Bits 23-21,15-12,10-0 */
	/* SETTLE_n,FILTER_MODE_n,FS_n */
	for (uint8_t i = 0; i < 8; i++)
	{
		filter_val = 0b111000000001000000001010;
		tx[0] = (filter_val >> 16) & 0xFF;
		tx[1] = (filter_val >> 8) & 0xFF;
		tx[2] = filter_val & 0xFF;
		ADC2_Write(AD4130_FILTER_0 + i, tx, 3);
	}
}

void ADC2_Channel_0(void)
{
	uint32_t channel_0_val;
	uint8_t tx[3];

	/* Bits 23,22-20,17-13,12-8,3-0 */
	/* ENABLE_0,SETUP_0,AINP_0,AINM_0,I_OUT0_CH_0 */
	channel_0_val = 0b101000000100001100000000;
	tx[0] = (channel_0_val >> 16) & 0xFF;
	tx[1] = (channel_0_val >> 8) & 0xFF;
	tx[2] = channel_0_val & 0xFF;
	ADC2_Write(AD4130_CHANNEL_0, tx, 3);
}
