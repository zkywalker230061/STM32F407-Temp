#ifndef SENSOR_READ_H
#define SENSOR_READ_H

#include "common/error_code.h"


#ifdef __cplusplus
extern "C" {
#endif

ErrorCode_t read_sensor(void);

void sensor_read_reset_synchronized_conversion(void);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_READ_H */
