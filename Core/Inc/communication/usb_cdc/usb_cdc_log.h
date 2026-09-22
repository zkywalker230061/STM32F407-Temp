#ifndef USB_CDC_LOG_H
#define USB_CDC_LOG_H

#include "application/usb_comm.h"

#ifdef __cplusplus
extern "C" {
#endif

int _write(
		int file,
		char *data,
		int length
);

#ifdef __cplusplus
}
#endif

#endif  /* USB_CDC_LOG_H */
