#ifndef BSP_AD4130_H
#define BSP_AD4130_H

#include <stdint.h>
#include "spi.h"
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */

typedef struct
{
	uint8_t id;
	uint8_t status;
	uint16_t error;
	float vref;
	float gain;
} AD4130InitResult_t;

#define AD4130_VREF		1.25f
#define AD4130_GAIN		128.0f

/* ------------------------------------------------------------------------ */

#define AD4130_STATUS         0x00  /* 8 bits */
#define AD4130_ADC_CONTROL    0x01  /* 16 bits */
#define AD4130_DATA           0x02  /* 24 bits */
#define AD4130_IO_CONTROL     0x03  /* 16 bits */
#define AD4130_VBIAS_CONTROL  0x04  /* 16 bits */
#define AD4130_ID             0x05  /* 8 bits */
#define AD4130_ERROR          0x06  /* 16 bits */
#define AD4130_ERROR_EN       0x07  /* 16 bits */
#define AD4130_MCLK_COUNT     0x08  /* 8 bits */

/* Channel Configuration Registers */
/* 24 bits */
#define AD4130_CHANNEL_0      0x09
#define AD4130_CHANNEL_1      0x0A
#define AD4130_CHANNEL_2      0x0B
#define AD4130_CHANNEL_3      0x0C
#define AD4130_CHANNEL_4      0x0D
#define AD4130_CHANNEL_5      0x0E
#define AD4130_CHANNEL_6      0x0F
#define AD4130_CHANNEL_7      0x10
#define AD4130_CHANNEL_8      0x11
#define AD4130_CHANNEL_9      0x12
#define AD4130_CHANNEL_10     0x13
#define AD4130_CHANNEL_11     0x14
#define AD4130_CHANNEL_12     0x15
#define AD4130_CHANNEL_13     0x16
#define AD4130_CHANNEL_14     0x17
#define AD4130_CHANNEL_15     0x18

/* Configuration Registers */
/* 16 bits */
#define AD4130_CONFIG_0       0x19
#define AD4130_CONFIG_1       0x1A
#define AD4130_CONFIG_2       0x1B
#define AD4130_CONFIG_3       0x1C
#define AD4130_CONFIG_4       0x1D
#define AD4130_CONFIG_5       0x1E
#define AD4130_CONFIG_6       0x1F
#define AD4130_CONFIG_7       0x20

/* Filter Registers */
/* 24 bits */
#define AD4130_FILTER_0       0x21
#define AD4130_FILTER_1       0x22
#define AD4130_FILTER_2       0x23
#define AD4130_FILTER_3       0x24
#define AD4130_FILTER_4       0x25
#define AD4130_FILTER_5       0x26
#define AD4130_FILTER_6       0x27
#define AD4130_FILTER_7       0x28

/* Offset Registers */
/* 24 bits */
#define AD4130_OFFSET_0       0x29
#define AD4130_OFFSET_1       0x2A
#define AD4130_OFFSET_2       0x2B
#define AD4130_OFFSET_3       0x2C
#define AD4130_OFFSET_4       0x2D
#define AD4130_OFFSET_5       0x2E
#define AD4130_OFFSET_6       0x2F
#define AD4130_OFFSET_7       0x30

/* Gain Registers */
/* 24 bits */
#define AD4130_GAIN_0         0x31
#define AD4130_GAIN_1         0x32
#define AD4130_GAIN_2         0x33
#define AD4130_GAIN_3         0x34
#define AD4130_GAIN_4         0x35
#define AD4130_GAIN_5         0x36
#define AD4130_GAIN_6         0x37
#define AD4130_GAIN_7         0x38

#define AD4130_MISC           0x39  /* 16 bits */
#define AD4130_FIFO_CONTROL   0x3A  /* 24 bits */
#define AD4130_FIFO_STATUS    0x3B  /* 8 bits */
#define AD4130_FIFO_THRESHOLD 0x3C  /* 24 bits */
#define AD4130_FIFO_DATA      0x3D  /* 24 bits */

/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef AD4130_Read_8_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint8_t *value
);
HAL_StatusTypeDef AD4130_Read_16_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint16_t *value
);
HAL_StatusTypeDef AD4130_Read_24_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint32_t *value
);
HAL_StatusTypeDef AD4130_Read_32_Bit(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		uint32_t *value
);
HAL_StatusTypeDef AD4130_Write(
		uint8_t adc_device_id,
		uint8_t reg_addr,
		const uint8_t *data,
		uint16_t len
);
HAL_StatusTypeDef AD4130_Reset(uint8_t adc_device_id);
HAL_StatusTypeDef AD4130_Init(
		uint8_t adc_device_id,
		AD4130InitResult_t *init_result
);

/* ------------------------------------------------------------------------ */

HAL_StatusTypeDef AD4130_Config(uint8_t adc_device_id);
HAL_StatusTypeDef AD4130_Filter(uint8_t adc_device_id);
HAL_StatusTypeDef AD4130_Channel_0(uint8_t adc_device_id);

/* ------------------------------------------------------------------------ */

#ifdef __cplusplus
}
#endif

#endif  /* BSP_AD4130_H */
