#ifndef SENSOR_COEFFS_STORAGE_H
#define SENSOR_COEFFS_STORAGE_H

#include <stdint.h>

#include "common/error_code.h"


#ifdef __cplusplus
extern "C" {
#endif

ErrorCode_t Sensor_Coeffs_Storage_Save(
		uint8_t adc_device_id,
		uint8_t channel,
		const uint8_t *binary_data,
		uint32_t binary_length
);

ErrorCode_t Sensor_Coeffs_Storage_Load(
		uint8_t adc_device_id,
		uint8_t channel,
		const uint8_t **binary_data,
		uint32_t *binary_length
);

ErrorCode_t Sensor_Coeffs_Storage_Erase(void);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_COEFFS_STORAGE_H */
