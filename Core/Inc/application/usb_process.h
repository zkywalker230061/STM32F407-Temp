#ifndef USB_PROCESS_H
#define USB_PROCESS_H

#include "application/sensor_coeffs.h"
#include "communication/usb_cdc/usb_cdc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USB_PROCESS_OK             0
#define USB_PROCESS_NOT_READY      1
#define USB_PROCESS_COEFFS_ERROR  -1
#define USB_PROCESS_COMMAND_ERROR -2

int usb_process(void);

#ifdef __cplusplus
}
#endif

#endif  /* USB_PROCESS_H */
