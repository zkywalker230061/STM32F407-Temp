#ifndef AD4130_MEASUREMENT_H
#define AD4130_MEASUREMENT_H

#include "drivers/ad4130.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AD4130_MEASUREMENT_OK                  0
#define AD4130_MEASUREMENT_NOT_READY           1
#define AD4130_MEASUREMENT_PARAM_ERROR        -1
#define AD4130_MEASUREMENT_CHANNEL_CONFIG_ERROR -2
#define AD4130_MEASUREMENT_IOUT_CONFIG_ERROR    -3
#define AD4130_MEASUREMENT_COMM_ERROR         -4
#define AD4130_MEASUREMENT_TIMEOUT            -5
#define AD4130_MEASUREMENT_STATUS_ERROR       -6
#define AD4130_MEASUREMENT_BELOW_RANGE        -7
#define AD4130_MEASUREMENT_ABOVE_RANGE        -8

int AD4130_Read_Resistance(
		uint8_t adc_device_id,
		uint8_t *channel,
		float *resistance
);

#ifdef __cplusplus
}
#endif

#endif  /* AD4130_MEASUREMENT_H */
