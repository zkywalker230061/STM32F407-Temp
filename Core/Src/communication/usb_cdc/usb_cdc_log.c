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
	(void)file;

	if ((data == NULL) || (length <= 0))
	{
		return 0;
	}

	(void)usb_comm_write((const uint8_t *)data, (uint16_t)length);
	return length;
}
