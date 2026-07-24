#ifndef CHEBYCHEV_H
#define CHEBYCHEV_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

float chebychev_point(float x, float zl, float zu, const float *coeffs, int order);

#ifdef __cplusplus
}
#endif

#endif  /* CHEBYCHEV_H */
