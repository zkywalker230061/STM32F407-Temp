#ifndef BSP_AD4130_H
#define BSP_AD4130_H

#include <stdint.h>
#include "spi.h"
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AD4130_ADC_CONTROL    0x01
#define AD4130_ID             0x05

void ADC1_Init(void);
uint8_t ADC1_Read_8_bit(uint8_t reg_addr);
uint16_t ADC1_Read_16_bit(uint8_t reg_addr);
void ADC1_Write(uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t ADC1_ReadID(void);

void ADC2_Init(void);
uint8_t ADC2_Read_8_bit(uint8_t reg_addr);
uint16_t ADC2_Read_16_bit(uint8_t reg_addr);
void ADC2_Write(uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t ADC2_ReadID(void);

#ifdef __cplusplus
}
#endif

#endif  /* BSP_AD4130_H */
