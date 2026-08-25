#ifndef SENSOR_FIT_H
#define SENSOR_FIT_H

#include <stdint.h>
#include <math.h>
#include "application/chebychev.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_FIT_OK          0
#define SENSOR_FIT_PARAM_ERROR -1
#define SENSOR_FIT_RANGE_ERROR -2

#define SENSOR_FIT_MAX_SEGMENTS		4
#define SENSOR_FIT_MAX_ORDER		15
#define SENSOR_FIT_COEFF_COUNT		(SENSOR_FIT_MAX_ORDER + 1)

typedef struct
{
	float z_min;  /* log10(R) lower bound (high temperature end) */
	float z_max;  /* log10(R) upper bound (low temperature end) */
	float r_left;  /* resistance near z_min */
	float r_right;  /* resistance near z_max */
	/* Temperature relationship: z_max(r) < r_left < r_right < z_min(r) */
	float coeffs[SENSOR_FIT_COEFF_COUNT];
	int order;
} CurveSegment_t;

typedef struct
{
	CurveSegment_t segments[SENSOR_FIT_MAX_SEGMENTS];
	uint16_t segment_count;
} Curve_t;

int resistance_to_temperature(float resistance, float *temperature);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_FIT_H */
