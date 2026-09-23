#ifndef USB_COMM_H
#define USB_COMM_H

#include <stdint.h>

#include "common/error_code.h"


#ifdef __cplusplus
extern "C" {
#endif

ErrorCode_t usb_comm_process(void);

uint8_t usb_comm_measurement_log_enabled(void);

ErrorCode_t usb_comm_write(
		const uint8_t *data,
		uint16_t length
);

#ifdef __cplusplus
}
#endif

#endif  /* USB_COMM_H */
