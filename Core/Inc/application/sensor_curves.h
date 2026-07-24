#ifndef SENSOR_CURVES_H
#define SENSOR_CURVES_H

#include <math.h>
#include "application/chebychev.h"

#define TEMP_ERROR  -999.9f

typedef struct {
	float z_min;  /* log10(R) lower bound (high temperature end) */
	float z_max;  /* log10(R) upper bound (low temperature end) */
	const float *coeffs;
	int order;
} CurveSegment_t;

#include "storage/sensor_coeffs.h"

#ifdef __cplusplus
extern "C" {
#endif

float resistance_to_temperature(float resistance);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_CURVES_H */
