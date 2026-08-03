#include "bsp/ad4130.h"

/* ADC1 (CS = PA4) */
/* ------------------------------------------------------------------------ */
void ADC1_Reset(void)
{
	uint8_t tx[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, tx, 8, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_SET);

	HAL_Delay(100);
}

void ADC1_Init(void)
{
	uint16_t control_val;
	uint16_t error_en_val;
	uint8_t tx[2];

	ADC1_Reset();

	/* Bit 9: CSB_EN */
	control_val = 0x0200;
	tx[0] = (control_val >> 8) & 0xFF;
	tx[1] = control_val & 0xFF;
	ADC1_Write(AD4130_ADC_CONTROL, tx, 2);

	/* Bits 3,4,5,6 */
	/* SPI_WRITE_ERROR_EN,SPI_READ_ERR_EN,SPI_SCLK_CNT_ERR_EN,SPI_IGNORE_ERR_EN */
	error_en_val = 0x0078;
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

	HAL_Delay(100);
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

	HAL_Delay(100);
}

void ADC2_Init(void) {
	uint16_t control_val;
	uint16_t error_en_val;
	uint8_t tx[2];

	ADC2_Reset();

	/* Bit 9: CSB_EN */
	control_val = 0x0200;
	tx[0] = (control_val >> 8) & 0xFF;
	tx[1] = control_val & 0xFF;
	ADC2_Write(AD4130_ADC_CONTROL, tx, 2);

	/* Bits 3,4,5,6 */
	/* SPI_WRITE_ERROR_EN,SPI_READ_ERR_EN,SPI_SCLK_CNT_ERR_EN,SPI_IGNORE_ERR_EN */
	error_en_val = 0x0078;
	tx[0] = (error_en_val >> 8) & 0xFF;
	tx[1] = error_en_val & 0xFF;
	ADC2_Write(AD4130_ERROR_EN, tx, 2);
}

uint8_t ADC2_Read_8_bit(uint8_t reg_addr)
{
	uint8_t tx_cache[2];
	uint8_t rx_cache[2];

	tx_cache[0] = 0b01000000 | reg_addr;  /* COMMS, read */
	tx_cache[1] = 0b00000000;

	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_cache, rx_cache, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin, GPIO_PIN_SET);

	return rx_cache[1];
}

uint16_t ADC2_Read_16_bit(uint8_t reg_addr)
{
	uint8_t tx_cache[3];
	uint8_t rx_cache[3];
	uint16_t rx;

	tx_cache[0] = 0b01000000 | reg_addr;  /* COMMS, read */
	tx_cache[1] = 0b00000000;
	tx_cache[2] = 0b00000000;

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

	HAL_Delay(100);
}

uint8_t ADC2_ReadID(void)
{
	return ADC2_Read_8_bit(AD4130_ID);
}
