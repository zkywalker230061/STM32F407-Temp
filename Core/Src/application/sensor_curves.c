#include "application/sensor_curves.h"

#define SEGMENT_COUNT 4

typedef struct {
	float z_min;  /* log10(R) lower bound (high temperature end) */
	float z_max;  /* log10(R) upper bound (low temperature end) */
	const float *coeffs;
	int order;
} CurveSegment_t;

/* -----TEMPORARY BEGINS----- */
/* Segment 1: 0.310 K - 3.40 K, R: 13270 -> 580 ohm */
static const float seg1_coeffs[] = {
	1.197068f, -1.418768f, 0.694578f, -0.300731f, 0.118594f,
	-0.043832f, 0.015200f, -0.004210f, 0.001395f, -0.000293f,
	0.000780f
};
/* Segment 2: 3.40 K - 25.9 K, R: 580 -> 196.1 ohm */
static const float seg2_coeffs[] = {
	12.189418f, -12.356477f, 3.832469f, -0.731453f, 0.040854f,
	0.021640f, -0.004292f, -0.001516f, 0.001042f, -0.000332f
};
/* Segment 3: 25.9 K - 160 K, R: 196.1 -> 64.75 ohm */
static const float seg3_coeffs[] = {
	85.818601f, -76.954857f, 15.436591f, -1.658131f, 0.089013f,
	-0.019841f, 0.019132f, -0.001219f
};
/* Segment 4: 160 K - 409 K, R: 64.75 -> 33.19 ohm */
static const float seg4_coeffs[] = {
	261.041252f, -138.753330f, 20.446563f, -3.222205f, 0.595623f,
	-0.112227f, 0.021719f, -0.008874f, -0.004645f
};

static const CurveSegment_t g_segments[SEGMENT_COUNT] = {
	{
		.z_min = log10f(580.0f),
		.z_max = log10f(13270.0f),
		.coeffs = seg1_coeffs,
		.order = 10
	},
	{
		.z_min = log10f(196.1f),
		.z_max = log10f(580.0f),
		.coeffs = seg2_coeffs,
		.order = 9
	},
	{
		.z_min = log10f(64.75f),
		.z_max = log10f(196.1f),
		.coeffs = seg3_coeffs,
		.order = 7
	},
	{
		.z_min = log10f(33.19f),
		.z_max = log10f(64.75f),
		.coeffs = seg4_coeffs,
		.order = 8
	}
};
/* -----TEMPORARY ENDS----- */


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
