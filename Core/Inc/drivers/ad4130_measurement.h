#ifndef AD4130_MEASUREMENT_H
#define AD4130_MEASUREMENT_H

#include "drivers/ad4130.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef AD4130_Read_Resistance(
		uint8_t adc_device_id,
		uint8_t *channel,
		float *resistance
);

#ifdef __cplusplus
}
#endif

#endif  /* AD4130_MEASUREMENT_H */
