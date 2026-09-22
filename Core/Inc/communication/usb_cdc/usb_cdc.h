#ifndef USB_CDC_H
#define USB_CDC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_CDC_OK              0
#define USB_CDC_NOT_READY       1
#define USB_CDC_PARAM_ERROR    -1
#define USB_CDC_FORMAT_ERROR   -2
#define USB_CDC_LENGTH_ERROR   -3
#define USB_CDC_BUSY           -4
#define USB_CDC_TRANSMIT_ERROR -5

typedef enum
{
	USB_CDC_COMMAND_NONE = 0,
	USB_CDC_COMMAND_SCUP,
	USB_CDC_COMMAND_RSET
} USB_CDC_Command_t;

extern volatile USB_CDC_Command_t usb_cdc_command;

int USB_CDC_Initialize(void);

int USB_CDC_Receive(
		const uint8_t *data,
		uint32_t length
);

int USB_CDC_Transmit(
		const uint8_t *data,
		uint16_t length
);

#ifdef __cplusplus
}
#endif

#endif  /* USB_CDC_H */
