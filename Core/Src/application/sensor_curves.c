#include "application/sensor_curves.h"

typedef struct {
	float z_min;  /* log10(R) lower bound (high temperature end) */
	float z_max;  /* log10(R) upper bound (low temperature end) */
	float r_left;  /* resistance near z_min */
	float r_right;  /* resistance near z_max */
	/* Temperature relationship: z_max(r) < r_left < r_right < z_min(r) */
	const float *coeffs;
	int order;
} CurveSegment_t;

#include "storage/sensor_coeffs.inc"

float resistance_to_temperature(float resistance)
{
	if (resistance <= 0.0f)
	{
		return TEMP_ERROR;
	}

	for (int i = 0; i < SEGMENT_COUNT; i++)
	{
		if (resistance >= segments[i].r_right
				&& resistance < segments[i].r_left)
		{
			return chebychev_point(
					resistance,
					segments[i].z_min, segments[i].z_max,
					segments[i].coeffs, segments[i].order
				);
		}
	}

	return TEMP_ERROR;
}
