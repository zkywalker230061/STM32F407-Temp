#include "application/chebychev.h"

float chebychev_point(
		float x, float zl, float zu,
		const float *coeffs, int order
) {
	float z = log10f(x);
	float k = ((z - zl) - (zu - z)) / (zu - zl);

	/* Clenshaw */
	float b1 = 0.0f; /* b_{N+1} */
	float b2 = 0.0f; /* b_{N+2} */

	for (int i = order; i >= 1; i--) {
		float b0 = 2.0f * k * b1 - b2 + coeffs[i];
		b2 = b1;
		b1 = b0;
	}

	return coeffs[0] + k * b1 - b2;
}
