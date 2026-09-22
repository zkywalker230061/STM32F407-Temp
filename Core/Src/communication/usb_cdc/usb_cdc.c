#include "communication/usb_cdc/usb_cdc.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "communication/usb_cdc/usb_cdc_scup.h"

#define USB_CDC_MAGIC_SIZE 4U

typedef enum
{
	USB_CDC_FRAME_NONE = 0,
	USB_CDC_FRAME_SCUP,
	USB_CDC_FRAME_RSET
} USB_CDC_Frame_t;

static uint8_t usb_cdc_magic[USB_CDC_MAGIC_SIZE];
static uint32_t usb_cdc_magic_length;
static USB_CDC_Frame_t usb_cdc_frame;

volatile USB_CDC_Command_t usb_cdc_command;

extern USBD_HandleTypeDef hUsbDeviceFS;

static void USB_CDC_Reset_Frame(void);
static int USB_CDC_Identify_Frame(void);
static int USB_CDC_Convert_SCUP_Result(int result);

int USB_CDC_Initialize(void)
{
	USB_CDC_Reset_Frame();
	USB_CDC_SCUP_Reset();
	usb_cdc_command = USB_CDC_COMMAND_NONE;

	return USB_CDC_OK;
}

int USB_CDC_Receive(
		const uint8_t *data,
		uint32_t length
)
{
	uint32_t data_index = 0U;
	int result;

	if (data == NULL)
	{
		return USB_CDC_PARAM_ERROR;
	}

	if (usb_cdc_frame == USB_CDC_FRAME_NONE)
	{
		if (usb_cdc_command != USB_CDC_COMMAND_NONE)
		{
			return USB_CDC_BUSY;
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
			return USB_CDC_NOT_READY;
		}

		result = USB_CDC_Identify_Frame();
		if (result != USB_CDC_OK)
		{
			USB_CDC_Reset_Frame();
			return result;
		}

		if (usb_cdc_frame == USB_CDC_FRAME_RSET)
		{
			USB_CDC_Reset_Frame();
			if (data_index < length)
			{
				return USB_CDC_LENGTH_ERROR;
			}
			usb_cdc_command = USB_CDC_COMMAND_RSET;
			return USB_CDC_OK;
		}

		result = USB_CDC_SCUP_Receive(usb_cdc_magic, USB_CDC_MAGIC_SIZE);
		if (result != USB_CDC_SCUP_NOT_READY)
		{
			USB_CDC_Reset_Frame();
			usb_cdc_command = USB_CDC_COMMAND_SCUP;
			return USB_CDC_Convert_SCUP_Result(result);
		}
	}

	if (data_index == length)
	{
		return USB_CDC_NOT_READY;
	}

	result = USB_CDC_SCUP_Receive(&data[data_index], length-data_index);
	if (result != USB_CDC_SCUP_NOT_READY)
	{
		USB_CDC_Reset_Frame();
		usb_cdc_command = USB_CDC_COMMAND_SCUP;
	}

	return USB_CDC_Convert_SCUP_Result(result);
}

int USB_CDC_Transmit(
		const uint8_t *data,
		uint16_t length
)
{
	uint8_t result;

	if ((data == NULL) || (length == 0U))
	{
		return USB_CDC_PARAM_ERROR;
	}

	if (
			(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
			|| (hUsbDeviceFS.pClassData == NULL)
	)
	{
		return USB_CDC_NOT_READY;
	}

	result = CDC_Transmit_FS((uint8_t *)data, length);
	if (result == USBD_OK)
	{
		return USB_CDC_OK;
	}
	if (result == USBD_BUSY)
	{
		return USB_CDC_BUSY;
	}

	return USB_CDC_TRANSMIT_ERROR;
}

static void USB_CDC_Reset_Frame(void)
{
	usb_cdc_magic_length = 0U;
	usb_cdc_frame = USB_CDC_FRAME_NONE;
}

static int USB_CDC_Identify_Frame(void)
{
	if (
			(usb_cdc_magic[0] == 'S')
			&& (usb_cdc_magic[1] == 'C')
			&& (usb_cdc_magic[2] == 'U')
			&& (usb_cdc_magic[3] == 'P')
	)
	{
		usb_cdc_frame = USB_CDC_FRAME_SCUP;
		return USB_CDC_OK;
	}

	if (
			(usb_cdc_magic[0] == 'R')
			&& (usb_cdc_magic[1] == 'S')
			&& (usb_cdc_magic[2] == 'E')
			&& (usb_cdc_magic[3] == 'T')
	)
	{
		usb_cdc_frame = USB_CDC_FRAME_RSET;
		return USB_CDC_OK;
	}

	return USB_CDC_FORMAT_ERROR;
}

static int USB_CDC_Convert_SCUP_Result(int result)
{
	switch (result)
	{
		case USB_CDC_SCUP_OK:
			return USB_CDC_OK;

		case USB_CDC_SCUP_NOT_READY:
			return USB_CDC_NOT_READY;

		case USB_CDC_SCUP_PARAM_ERROR:
			return USB_CDC_PARAM_ERROR;

		case USB_CDC_SCUP_FORMAT_ERROR:
			return USB_CDC_FORMAT_ERROR;

		case USB_CDC_SCUP_LENGTH_ERROR:
			return USB_CDC_LENGTH_ERROR;

		case USB_CDC_SCUP_STATE_ERROR:
			return USB_CDC_BUSY;

		default:
			return USB_CDC_FORMAT_ERROR;
	}
}
