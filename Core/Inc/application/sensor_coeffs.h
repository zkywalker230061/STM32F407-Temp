#ifndef SENSOR_COEFFS_H
#define SENSOR_COEFFS_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "communication/sensor_coeffs_transfer.h"
#include "storage/sensor_coeffs_decoder.h"
#include "storage/sensor_coeffs_format.h"
#include "storage/sensor_coeffs_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_COEFFS_OK                  0
#define SENSOR_COEFFS_NOT_READY           1
#define SENSOR_COEFFS_UPDATED             2
#define SENSOR_COEFFS_DECODE_ERROR       -2
#define SENSOR_COEFFS_STORAGE_LOAD_ERROR -3
#define SENSOR_COEFFS_STORAGE_SAVE_ERROR -4
#define SENSOR_COEFFS_TRANSFER_ERROR     -5

int sensor_coeffs_initialize(void);

int sensor_coeffs_process(void);

const Curve_t *sensor_coeffs_get_curve(
		uint8_t adc_device_id,
		uint8_t channel_id
);

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_COEFFS_H */
