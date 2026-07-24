#include "application/sensor_curves.h"

float resistance_to_temperature(float resistance) {
	if (resistance <= 0.0f) {
		return TEMP_ERROR;
	}

	float z = log10f(resistance);

	for (int i = 0; i < SEGMENT_COUNT; i++) {
		if (z >= g_segments[i].z_min && z < g_segments[i].z_max) {
			return chebychev_point(
					resistance,
					g_segments[i].z_min, g_segments[i].z_max,
					g_segments[i].coeffs, g_segments[i].order
				);
		}
	}

	return TEMP_ERROR;
}
