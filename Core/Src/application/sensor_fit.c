#include "application/sensor_fit.h"

#include <math.h>
#include <stddef.h>

#include "application/chebychev.h"
#include "storage/sensor_coeffs.inc"


static ErrorCode_t resistance_to_temperature_from_inc(
		float resistance,
		float *temperature
)
{
	if (temperature == NULL)
	{
		return ERROR_CODE_SENSOR_FIT_ILLEGAL_PARAM;
	}
	if ((!isfinite(resistance)) || (resistance <= 0.0f))
	{
		return ERROR_CODE_SENSOR_FIT_ILLEGAL_RESISTANCE;
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

			return ERROR_CODE_NONE;
		}
	}

	return ERROR_CODE_SENSOR_FIT_RANGE;
}

static ErrorCode_t resistance_to_temperature_from_curve(
		float resistance,
		const Curve_t *curve,
		float *temperature
)
{
	if ((curve == NULL) || (temperature == NULL))
	{
		return ERROR_CODE_SENSOR_FIT_ILLEGAL_PARAM;
	}
	if ((!isfinite(resistance)) || (resistance <= 0.0f))
	{
		return ERROR_CODE_SENSOR_FIT_ILLEGAL_RESISTANCE;
	}
	*temperature = 0.0f;

	for (uint16_t i = 0; i < curve->segment_count; i++)
	{
		if (
				(resistance >= curve->segments[i].r_right)
				&& (resistance < curve->segments[i].r_left)
		)
		{
			*temperature = chebychev_point(
					resistance,
					curve->segments[i].z_min,
					curve->segments[i].z_max,
					curve->segments[i].coeffs,
					curve->segments[i].order
			);

			return ERROR_CODE_NONE;
		}
	}

	return ERROR_CODE_SENSOR_FIT_RANGE;
}

ErrorCode_t resistance_to_temperature(
		float resistance,
		const Curve_t *curve,
		float *temperature
)
{
	if (curve == NULL)
	{
		return resistance_to_temperature_from_inc(
				resistance,
				temperature
		);
	}

	return resistance_to_temperature_from_curve(
			resistance,
			curve,
			temperature
	);
}
