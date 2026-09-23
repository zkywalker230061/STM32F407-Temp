#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>

#include "common/error_code.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	USB_CDC_COMMAND_NONE = 0,
	USB_CDC_COMMAND_RSET,
	USB_CDC_COMMAND_SCUP,
	USB_CDC_COMMAND_CRSC,
	USB_CDC_COMMAND_LOGE,
	USB_CDC_COMMAND_LOGD
} USB_CDC_Command_t;

extern volatile USB_CDC_Command_t usb_cdc_command;
extern volatile ErrorCode_t usb_cdc_receive_error;


void USB_CDC_Initialize(void);

ErrorCode_t USB_CDC_Receive(
		const uint8_t *data,
		uint32_t length
);

ErrorCode_t USB_CDC_Transmit(
		const uint8_t *data,
		uint16_t length
);

uint8_t USB_CDC_Transmit_Ready(void);

uint8_t USB_CDC_Transmit_Busy(void);

void USB_CDC_Transmit_Complete(void);

#ifdef __cplusplus
}
#endif

#endif  /* USB_CDC_H */
