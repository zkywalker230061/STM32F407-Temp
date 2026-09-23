#ifndef SENSOR_COEFFS_H
#define SENSOR_COEFFS_H

#include <stdint.h>

#include "application/sensor_fit.h"
#include "common/error_code.h"


#ifdef __cplusplus
extern "C" {
#endif

ErrorCode_t sensor_coeffs_initialize(void);

ErrorCode_t sensor_coeffs_process(void);

const Curve_t *sensor_coeffs_get_curve(
		uint8_t adc_device_id,
		uint8_t channel
);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_COEFFS_H */
