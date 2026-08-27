#ifndef SENSOR_COEFFS_TRANSFER_H
#define SENSOR_COEFFS_TRANSFER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_COEFFS_TRANSFER_OK            0
#define SENSOR_COEFFS_TRANSFER_NOT_READY     1
#define SENSOR_COEFFS_TRANSFER_PARAM_ERROR  -1
#define SENSOR_COEFFS_TRANSFER_FORMAT_ERROR -2
#define SENSOR_COEFFS_TRANSFER_LENGTH_ERROR -3
#define SENSOR_COEFFS_TRANSFER_STATE_ERROR  -4

int Sensor_Coeffs_Transfer_Receive(
		const uint8_t *data,
		uint32_t length
);

int Sensor_Coeffs_Transfer_Get_Data(
		uint8_t *adc_device_id,
		uint8_t *channel,
		const uint8_t **binary_data,
		uint32_t *binary_length
);

void Sensor_Coeffs_Transfer_Reset(void);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_COEFFS_TRANSFER_H */
