#ifndef SENSOR_FIT_H
#define SENSOR_FIT_H

#include <stdint.h>

#include "common/error_code.h"
#include "storage/sensor_coeffs_format.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	float z_min;  /* log10(R) lower bound (high temperature end) */
	float z_max;  /* log10(R) upper bound (low temperature end) */
	float r_left;  /* resistance near z_min */
	float r_right;  /* resistance near z_max */
	/* Temperature relationship: z_max(r) < r_left < r_right < z_min(r) */
	float coeffs[SENSOR_COEFFS_FORMAT_COEFF_COUNT];
	int order;
} CurveSegment_t;

typedef struct
{
	CurveSegment_t segments[SENSOR_COEFFS_FORMAT_MAX_SEGMENTS];
	uint16_t segment_count;
} Curve_t;


ErrorCode_t resistance_to_temperature(
		float resistance,
		const Curve_t *curve,
		float *temperature
);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_FIT_H */
