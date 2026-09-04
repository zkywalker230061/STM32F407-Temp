#ifndef SENSOR_COEFFS_DECODER_H
#define SENSOR_COEFFS_DECODER_H

#include <stdint.h>
#include "application/sensor_fit.h"
#include "storage/sensor_coeffs_format.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_COEFFS_DECODE_OK             0
#define SENSOR_COEFFS_DECODE_PARAM_ERROR   -1
#define SENSOR_COEFFS_DECODE_FORMAT_ERROR  -2
#define SENSOR_COEFFS_DECODE_VERSION_ERROR -3
#define SENSOR_COEFFS_DECODE_CRC_ERROR     -4

int Sensor_Coeffs_Decode(
		const uint8_t *data,
		uint32_t length,
		Curve_t *curve
);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_COEFFS_DECODER_H */
