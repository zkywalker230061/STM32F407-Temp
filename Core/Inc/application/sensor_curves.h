#ifndef SENSOR_CURVES_H
#define SENSOR_CURVES_H

#include <math.h>
#include "application/chebychev.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TEMP_ERROR  -999.9f

float resistance_to_temperature(float resistance);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_CURVES_H */
