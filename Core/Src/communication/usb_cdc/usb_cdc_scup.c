#include "communication/usb_cdc/usb_cdc_scup.h"

#include <stddef.h>

#include "storage/sensor_coeffs_format.h"


#define USB_CDC_SCUP_HEADER_SIZE		12U
#define USB_CDC_SCUP_FRAME_MAX_SIZE	360U

#define USB_CDC_SCUP_RECEIVING		0U
#define USB_CDC_SCUP_READY			1U
#define USB_CDC_SCUP_ERROR			2U

static uint8_t usb_cdc_scup_buffer[USB_CDC_SCUP_FRAME_MAX_SIZE];
static uint32_t usb_cdc_scup_received_length;
static volatile uint32_t usb_cdc_scup_binary_length;
static volatile uint8_t usb_cdc_scup_state;
static volatile int usb_cdc_scup_error;


static uint32_t USB_CDC_SCUP_Read_32_Bit(const uint8_t *data)
{
	return (
			(uint32_t)data[0]
			| ((uint32_t)data[1] << 8)
			| ((uint32_t)data[2] << 16)
			| ((uint32_t)data[3] << 24)
	);
}

int USB_CDC_SCUP_Receive(
		const uint8_t *data,
		uint32_t length
)
{
	uint32_t remaining_length;
	uint32_t expected_length;

	if (data == NULL)
	{
		return USB_CDC_SCUP_PARAM_ERROR;
	}

	if (usb_cdc_scup_state != USB_CDC_SCUP_RECEIVING)
	{
		return USB_CDC_SCUP_STATE_ERROR;
	}

	remaining_length = USB_CDC_SCUP_FRAME_MAX_SIZE - usb_cdc_scup_received_length;
	if (length > remaining_length)
	{
		usb_cdc_scup_error = USB_CDC_SCUP_LENGTH_ERROR;
		usb_cdc_scup_state = USB_CDC_SCUP_ERROR;
		return USB_CDC_SCUP_LENGTH_ERROR;
	}

	for (uint32_t i = 0U; i < length; i++)
	{
		usb_cdc_scup_buffer[usb_cdc_scup_received_length + i] = data[i];
	}
	usb_cdc_scup_received_length += length;

	if (usb_cdc_scup_received_length < USB_CDC_SCUP_HEADER_SIZE)
	{
		return USB_CDC_SCUP_NOT_READY;
	}

	if (
			(usb_cdc_scup_buffer[0] != 'S')
			|| (usb_cdc_scup_buffer[1] != 'C')
			|| (usb_cdc_scup_buffer[2] != 'U')
			|| (usb_cdc_scup_buffer[3] != 'P')
			|| (usb_cdc_scup_buffer[4] < 1U)
			|| (usb_cdc_scup_buffer[4] > 2U)
			|| (usb_cdc_scup_buffer[5] > 3U)
			|| (usb_cdc_scup_buffer[6] != 0U)
			|| (usb_cdc_scup_buffer[7] != 0U)
	)
	{
		usb_cdc_scup_error = USB_CDC_SCUP_FORMAT_ERROR;
		usb_cdc_scup_state = USB_CDC_SCUP_ERROR;
		return USB_CDC_SCUP_FORMAT_ERROR;
	}

	usb_cdc_scup_binary_length = USB_CDC_SCUP_Read_32_Bit(&usb_cdc_scup_buffer[8]);
	if (
			(usb_cdc_scup_binary_length < SENSOR_COEFFS_BINARY_MIN_SIZE)
			|| (usb_cdc_scup_binary_length > SENSOR_COEFFS_BINARY_MAX_SIZE)
	)
	{
		usb_cdc_scup_error = USB_CDC_SCUP_LENGTH_ERROR;
		usb_cdc_scup_state = USB_CDC_SCUP_ERROR;
		return USB_CDC_SCUP_LENGTH_ERROR;
	}

	expected_length = USB_CDC_SCUP_HEADER_SIZE + usb_cdc_scup_binary_length;
	if (usb_cdc_scup_received_length > expected_length)
	{
		usb_cdc_scup_error = USB_CDC_SCUP_LENGTH_ERROR;
		usb_cdc_scup_state = USB_CDC_SCUP_ERROR;
		return USB_CDC_SCUP_LENGTH_ERROR;
	}

	if (usb_cdc_scup_received_length < expected_length)
	{
		return USB_CDC_SCUP_NOT_READY;
	}

	usb_cdc_scup_state = USB_CDC_SCUP_READY;

	return USB_CDC_SCUP_OK;
}

int USB_CDC_SCUP_Get_Data(
		uint8_t *adc_device_id,
		uint8_t *channel,
		const uint8_t **binary_data,
		uint32_t *binary_length
)
{
	if (
			(adc_device_id == NULL) || (channel == NULL)
			|| (binary_data == NULL) || (binary_length == NULL)
	)
	{
		return USB_CDC_SCUP_PARAM_ERROR;
	}

	*adc_device_id = 0U;
	*channel = 0xFFU;
	*binary_data = NULL;
	*binary_length = 0U;

	if (usb_cdc_scup_state == USB_CDC_SCUP_ERROR)
	{
		return usb_cdc_scup_error;
	}

	if (usb_cdc_scup_state != USB_CDC_SCUP_READY)
	{
		return USB_CDC_SCUP_NOT_READY;
	}

	*adc_device_id = usb_cdc_scup_buffer[4];
	*channel = usb_cdc_scup_buffer[5];
	*binary_data = &usb_cdc_scup_buffer[USB_CDC_SCUP_HEADER_SIZE];
	*binary_length = usb_cdc_scup_binary_length;

	return USB_CDC_SCUP_OK;
}

void USB_CDC_SCUP_Reset(void)
{
	usb_cdc_scup_received_length = 0U;
	usb_cdc_scup_binary_length = 0U;
	usb_cdc_scup_error = USB_CDC_SCUP_OK;
	usb_cdc_scup_state = USB_CDC_SCUP_RECEIVING;
}
