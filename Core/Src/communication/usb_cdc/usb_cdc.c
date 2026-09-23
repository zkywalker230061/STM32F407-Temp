#include "communication/usb_cdc/usb_cdc.h"

#include <stddef.h>

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "communication/usb_cdc/usb_cdc_scup.h"


#define USB_CDC_MAGIC_SIZE 4U

typedef enum
{
	USB_CDC_FRAME_NONE = 0,
	USB_CDC_FRAME_RSET,
	USB_CDC_FRAME_SCUP,
	USB_CDC_FRAME_CRSC,
	USB_CDC_FRAME_LOGE,
	USB_CDC_FRAME_LOGD
} USB_CDC_Frame_t;

static uint8_t usb_cdc_magic[USB_CDC_MAGIC_SIZE];
static uint32_t usb_cdc_magic_length;
static USB_CDC_Frame_t usb_cdc_frame;
static volatile uint8_t usb_cdc_transmit_busy;

volatile USB_CDC_Command_t usb_cdc_command;
volatile ErrorCode_t usb_cdc_receive_error;

extern USBD_HandleTypeDef hUsbDeviceFS;

static void USB_CDC_Reset_Frame(void);
static ErrorCode_t USB_CDC_Identify_Frame(void);


void USB_CDC_Initialize(void)
{
	USB_CDC_Reset_Frame();
	USB_CDC_SCUP_Reset();
	usb_cdc_command = USB_CDC_COMMAND_NONE;
	usb_cdc_receive_error = ERROR_CODE_NONE;
	usb_cdc_transmit_busy = 0U;
}

ErrorCode_t USB_CDC_Receive(
		const uint8_t *data,
		uint32_t length
)
{
	uint32_t data_index = 0U;
	ErrorCode_t result;

	if (data == NULL)
	{
		usb_cdc_receive_error = ERROR_CODE_USB_CDC_ILLEGAL_PARAM;
		return usb_cdc_receive_error;
	}

	if (usb_cdc_frame == USB_CDC_FRAME_NONE)
	{
		if (usb_cdc_command != USB_CDC_COMMAND_NONE)
		{
			usb_cdc_receive_error = ERROR_CODE_USB_CDC_BUSY;
			return usb_cdc_receive_error;
		}

		while (
				(usb_cdc_magic_length < USB_CDC_MAGIC_SIZE)
				&& (data_index < length)
		)
		{
			usb_cdc_magic[usb_cdc_magic_length] = data[data_index];
			usb_cdc_magic_length++;
			data_index++;
		}

		if (usb_cdc_magic_length < USB_CDC_MAGIC_SIZE)
		{
			return ERROR_CODE_USB_CDC_RECEIVING;
		}

		result = USB_CDC_Identify_Frame();
		if (result != ERROR_CODE_NONE)
		{
			USB_CDC_Reset_Frame();
			usb_cdc_receive_error = result;
			return usb_cdc_receive_error;
		}

		if (usb_cdc_frame == USB_CDC_FRAME_RSET)
		{
			USB_CDC_Reset_Frame();
			if (data_index < length)
			{
				usb_cdc_receive_error = ERROR_CODE_USB_CDC_ILLEGAL_LENGTH;
				return usb_cdc_receive_error;
			}
			usb_cdc_command = USB_CDC_COMMAND_RSET;
			return ERROR_CODE_NONE;
		}

		if (usb_cdc_frame == USB_CDC_FRAME_SCUP)
		{
			result = USB_CDC_SCUP_Receive(usb_cdc_magic, USB_CDC_MAGIC_SIZE);
			if (result != ERROR_CODE_COEFFS_TRANSFER_NOT_READY)
			{
				USB_CDC_Reset_Frame();
				usb_cdc_command = USB_CDC_COMMAND_SCUP;
				return result;
			}
		}

		if (usb_cdc_frame == USB_CDC_FRAME_CRSC)
		{
			USB_CDC_Reset_Frame();
			if (data_index < length)
			{
				usb_cdc_receive_error = ERROR_CODE_USB_CDC_ILLEGAL_LENGTH;
				return usb_cdc_receive_error;
			}
			usb_cdc_command = USB_CDC_COMMAND_CRSC;
			return ERROR_CODE_NONE;
		}

		if (usb_cdc_frame == USB_CDC_FRAME_LOGE)
		{
			USB_CDC_Reset_Frame();
			if (data_index < length)
			{
				usb_cdc_receive_error = ERROR_CODE_USB_CDC_ILLEGAL_LENGTH;
				return usb_cdc_receive_error;
			}
			usb_cdc_command = USB_CDC_COMMAND_LOGE;
			return ERROR_CODE_NONE;
		}

		if (usb_cdc_frame == USB_CDC_FRAME_LOGD)
		{
			USB_CDC_Reset_Frame();
			if (data_index < length)
			{
				usb_cdc_receive_error = ERROR_CODE_USB_CDC_ILLEGAL_LENGTH;
				return usb_cdc_receive_error;
			}
			usb_cdc_command = USB_CDC_COMMAND_LOGD;
			return ERROR_CODE_NONE;
		}
	}

	if (data_index == length)
	{
		return ERROR_CODE_COEFFS_TRANSFER_NOT_READY;
	}

	result = USB_CDC_SCUP_Receive(&data[data_index], length-data_index);
	if (result != ERROR_CODE_COEFFS_TRANSFER_NOT_READY)
	{
		USB_CDC_Reset_Frame();
		usb_cdc_command = USB_CDC_COMMAND_SCUP;
	}

	return result;
}

static void USB_CDC_Reset_Frame(void)
{
	usb_cdc_magic_length = 0U;
	usb_cdc_frame = USB_CDC_FRAME_NONE;
}

static ErrorCode_t USB_CDC_Identify_Frame(void)
{
	if (
			(usb_cdc_magic[0] == 'R')
			&& (usb_cdc_magic[1] == 'S')
			&& (usb_cdc_magic[2] == 'E')
			&& (usb_cdc_magic[3] == 'T')
	)
	{
		usb_cdc_frame = USB_CDC_FRAME_RSET;
		return ERROR_CODE_NONE;
	}

	if (
			(usb_cdc_magic[0] == 'S')
			&& (usb_cdc_magic[1] == 'C')
			&& (usb_cdc_magic[2] == 'U')
			&& (usb_cdc_magic[3] == 'P')
	)
	{
		usb_cdc_frame = USB_CDC_FRAME_SCUP;
		return ERROR_CODE_NONE;
	}

	if (
			(usb_cdc_magic[0] == 'C')
			&& (usb_cdc_magic[1] == 'R')
			&& (usb_cdc_magic[2] == 'S')
			&& (usb_cdc_magic[3] == 'C')
	)
	{
		usb_cdc_frame = USB_CDC_FRAME_CRSC;
		return ERROR_CODE_NONE;
	}

	if (
			(usb_cdc_magic[0] == 'L')
			&& (usb_cdc_magic[1] == 'O')
			&& (usb_cdc_magic[2] == 'G')
			&& (usb_cdc_magic[3] == 'E')
	)
	{
		usb_cdc_frame = USB_CDC_FRAME_LOGE;
		return ERROR_CODE_NONE;
	}

	if (
			(usb_cdc_magic[0] == 'L')
			&& (usb_cdc_magic[1] == 'O')
			&& (usb_cdc_magic[2] == 'G')
			&& (usb_cdc_magic[3] == 'D')
	)
	{
		usb_cdc_frame = USB_CDC_FRAME_LOGD;
		return ERROR_CODE_NONE;
	}

	return ERROR_CODE_USB_CDC_ILLEGAL_FORMAT;
}

ErrorCode_t USB_CDC_Transmit(
		const uint8_t *data,
		uint16_t length
)
{
	uint8_t result;

	if ((data == NULL) || (length == 0U))
	{
		return ERROR_CODE_USB_CDC_ILLEGAL_PARAM;
	}

	if (USB_CDC_Transmit_Ready() == 0U)
	{
		return ERROR_CODE_USB_CDC_NOT_READY;
	}

	usb_cdc_transmit_busy = 1U;
	result = CDC_Transmit_FS((uint8_t *)data, length);
	if (result == USBD_OK)
	{
		return ERROR_CODE_NONE;
	}
	usb_cdc_transmit_busy = 0U;
	if (result == USBD_BUSY)
	{
		return ERROR_CODE_USB_CDC_BUSY;
	}

	return ERROR_CODE_USB_CDC_TRANSMIT;
}

uint8_t USB_CDC_Transmit_Ready(void)
{
	if (
			(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
			&& (hUsbDeviceFS.pClassData != NULL)
	)
	{
		return 1U;
	}

	return 0U;
}

uint8_t USB_CDC_Transmit_Busy(void)
{
	return usb_cdc_transmit_busy;
}

void USB_CDC_Transmit_Complete(void)
{
	usb_cdc_transmit_busy = 0U;
}
