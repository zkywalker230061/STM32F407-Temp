#include "communication/usb_cdc/usb_cdc_log.h"

#include <stddef.h>
#include <stdint.h>

#include "application/usb_comm.h"


int _write(
		int file,
		char *data,
		int length
)
{
	ErrorCode_t result;

	(void)file;

	if ((data == NULL) || (length <= 0))
	{
		return 0;
	}

	result = usb_comm_write((const uint8_t *)data, (uint16_t)length);
	if (
			(result != ERROR_CODE_NONE)
			&& (result != ERROR_CODE_USB_COMM_NOT_READY)
	)
	{
		return 0;
	}

	return length;
}
