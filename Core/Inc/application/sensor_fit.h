#ifndef SENSOR_FIT_H
#define SENSOR_FIT_H

#include <math.h>
#include "application/chebychev.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_FIT_OK          0
#define SENSOR_FIT_PARAM_ERROR -1
#define SENSOR_FIT_RANGE_ERROR -2

int resistance_to_temperature(float resistance, float *temperature);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_FIT_H */
