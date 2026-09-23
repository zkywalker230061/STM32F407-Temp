#ifndef AD4130_MEASUREMENT_H
#define AD4130_MEASUREMENT_H

#include <stdint.h>

#include "common/error_code.h"


#ifdef __cplusplus
extern "C" {
#endif

ErrorCode_t AD4130_Read_Resistance(
		uint8_t adc_device_id,
		uint8_t *channel,
		float *resistance
);

#ifdef __cplusplus
}
#endif

#endif  /* AD4130_MEASUREMENT_H */
