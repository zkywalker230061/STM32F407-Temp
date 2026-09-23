#ifndef SENSOR_COEFFS_DECODER_H
#define SENSOR_COEFFS_DECODER_H

#include <stdint.h>

#include "application/sensor_fit.h"
#include "common/error_code.h"


#ifdef __cplusplus
extern "C" {
#endif

ErrorCode_t Sensor_Coeffs_Decode(
		const uint8_t *data,
		uint32_t length,
		Curve_t *curve
);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_COEFFS_DECODER_H */
