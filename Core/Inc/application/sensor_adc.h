#ifndef SENSOR_ADC_H
#define SENSOR_ADC_H

#include <stdio.h>
#include "drivers/ad4130.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_ADC_OK            0
#define SENSOR_ADC_INIT_ERROR   -1
#define SENSOR_ADC_ID_ERROR     -2
#define SENSOR_ADC_SETUP_ERROR  -3

int sensor_adc_initialize(void);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_ADC_H */
