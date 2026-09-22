#include "application/usb_process.h"

int usb_process(void)
{
	USB_CDC_Command_t command;
	int coeffs_result;

	command = usb_cdc_command;
	if (command == USB_CDC_COMMAND_NONE)
	{
		return USB_PROCESS_NOT_READY;
	}

	switch (command)
	{
		case USB_CDC_COMMAND_SCUP:
			coeffs_result = sensor_coeffs_process();
			if (coeffs_result == SENSOR_COEFFS_NOT_READY)
			{
				return USB_PROCESS_NOT_READY;
			}

			usb_cdc_command = USB_CDC_COMMAND_NONE;
			if (coeffs_result == SENSOR_COEFFS_UPDATED)
			{
				NVIC_SystemReset();
				return USB_PROCESS_OK;
			}
			return USB_PROCESS_COEFFS_ERROR;

		case USB_CDC_COMMAND_RSET:
			/* Reserved for future implementation. */
			usb_cdc_command = USB_CDC_COMMAND_NONE;
			break;

		case USB_CDC_COMMAND_NONE:
		default:
			usb_cdc_command = USB_CDC_COMMAND_NONE;
			return USB_PROCESS_COMMAND_ERROR;
	}

	return USB_PROCESS_OK;
}
