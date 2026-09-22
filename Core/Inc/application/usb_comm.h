#ifndef USB_COMM_H
#define USB_COMM_H

#include <stddef.h>
#include <stdint.h>
#include "application/sensor_coeffs.h"
#include "communication/usb_cdc/usb_cdc.h"
#include "storage/sensor_coeffs_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USB_COMM_OK              0
#define USB_COMM_NOT_READY       1
#define USB_COMM_COEFFS_ERROR   -1
#define USB_COMM_COMMAND_ERROR  -2
#define USB_COMM_PARAM_ERROR    -3
#define USB_COMM_LENGTH_ERROR   -4
#define USB_COMM_BUSY           -5
#define USB_COMM_TRANSMIT_ERROR -6

int usb_comm_process(void);

uint8_t usb_comm_measurement_log_enabled(void);

int usb_comm_write(
		const uint8_t *data,
		uint16_t length
);

#ifdef __cplusplus
}
#endif

#endif  /* USB_COMM_H */
