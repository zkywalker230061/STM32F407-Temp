#include "application/sensor_fit.h"

#include "storage/sensor_coeffs.inc"

int resistance_to_temperature(float resistance, float *temperature)
{
	if ((resistance <= 0.0f) || (temperature == NULL))
	{
		return SENSOR_FIT_PARAM_ERROR;
	}
	*temperature = 0.0f;

	for (int i = 0; i < SEGMENT_COUNT; i++)
	{
		if (resistance >= segments[i].r_right && resistance < segments[i].r_left)
		{
			*temperature = chebychev_point(
					resistance,
					segments[i].z_min, segments[i].z_max,
					segments[i].coeffs, segments[i].order
			);

			return SENSOR_FIT_OK;
		}
	}

	return SENSOR_FIT_RANGE_ERROR;
}
