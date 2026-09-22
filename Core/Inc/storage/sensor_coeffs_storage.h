#ifndef SENSOR_COEFFS_STORAGE_H
#define SENSOR_COEFFS_STORAGE_H

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_COEFFS_STORAGE_OK            0
#define SENSOR_COEFFS_STORAGE_NOT_FOUND     1
#define SENSOR_COEFFS_STORAGE_PARAM_ERROR  -1
#define SENSOR_COEFFS_STORAGE_LENGTH_ERROR -2
#define SENSOR_COEFFS_STORAGE_ERASE_ERROR  -3
#define SENSOR_COEFFS_STORAGE_WRITE_ERROR  -4
#define SENSOR_COEFFS_STORAGE_VERIFY_ERROR -5
#define SENSOR_COEFFS_STORAGE_FULL_ERROR   -6


int Sensor_Coeffs_Storage_Save(
		uint8_t adc_device_id,
		uint8_t channel_id,
		const uint8_t *binary_data,
		uint32_t binary_length
);

int Sensor_Coeffs_Storage_Load(
		uint8_t adc_device_id,
		uint8_t channel_id,
		const uint8_t **binary_data,
		uint32_t *binary_length
);

int Sensor_Coeffs_Storage_Erase(void);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_COEFFS_STORAGE_H */
