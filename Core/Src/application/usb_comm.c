#include "application/usb_comm.h"

#include <stddef.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"

#include "application/sensor_coeffs.h"
#include "communication/usb_cdc/usb_cdc.h"
#include "storage/sensor_coeffs_storage.h"


#define USB_COMM_TX_BUFFER_COUNT 32U
#define USB_COMM_TX_BUFFER_SIZE  256U
#define USB_COMM_TX_BATCH_SIZE   (USB_COMM_TX_BUFFER_COUNT * USB_COMM_TX_BUFFER_SIZE)

static uint8_t usb_comm_tx_buffer[USB_COMM_TX_BUFFER_COUNT][USB_COMM_TX_BUFFER_SIZE];
static uint16_t usb_comm_tx_length[USB_COMM_TX_BUFFER_COUNT];
static uint8_t usb_comm_tx_batch[USB_COMM_TX_BATCH_SIZE];
static uint8_t usb_comm_tx_read_index;
static uint8_t usb_comm_tx_write_index;
static uint8_t usb_comm_tx_count;
static uint8_t usb_comm_tx_transmitting;
static uint8_t usb_comm_measurement_log_enable;
static uint8_t usb_comm_reset_pending;

static void usb_comm_reset_transmit(void);


int usb_comm_process(void)
{
	USB_CDC_Command_t command;
	uint8_t batch_count;
	uint8_t batch_read_index;
	uint16_t batch_length;
	uint16_t message_length;
	int command_result;
	int transmit_result;

	/* if reset */
	if (usb_comm_reset_pending != 0U)
	{
		command = USB_CDC_COMMAND_NONE;
	}
	else
	{
		command = usb_cdc_command;
	}

	/* process received */
	switch (command)
	{
		case USB_CDC_COMMAND_RSET:
		{
			usb_cdc_command = USB_CDC_COMMAND_NONE;
			printf("System reset\r\n");
			usb_comm_reset_pending = 1U;
			command_result = USB_COMM_OK;
			break;
		}

		case USB_CDC_COMMAND_SCUP:
		{
			int coeffs_result;

			coeffs_result = sensor_coeffs_process();
			if (coeffs_result == SENSOR_COEFFS_NOT_READY)
			{
				command_result = USB_COMM_NOT_READY;
				break;
			}

			usb_cdc_command = USB_CDC_COMMAND_NONE;
			if (coeffs_result == SENSOR_COEFFS_UPDATED)
			{
				printf("Sensor coefficients updated\r\n");
				usb_comm_reset_pending = 1U;
				command_result = USB_COMM_OK;
				break;
			}
			return USB_COMM_COEFFS_ERROR;
		}

		case USB_CDC_COMMAND_CRSC:
		{
			int storage_result;

			storage_result = Sensor_Coeffs_Storage_Erase();
			usb_cdc_command = USB_CDC_COMMAND_NONE;
			if (storage_result == SENSOR_COEFFS_STORAGE_OK)
			{
				printf("Sensor coefficients cleared\r\n");
				usb_comm_reset_pending = 1U;
				command_result = USB_COMM_OK;
				break;
			}
			return USB_COMM_COEFFS_ERROR;
		}

		case USB_CDC_COMMAND_LOGE:
		{
			usb_comm_measurement_log_enable = 1U;
			usb_cdc_command = USB_CDC_COMMAND_NONE;
			printf("Measurement log enabled\r\n");
			command_result = USB_COMM_OK;
			break;
		}

		case USB_CDC_COMMAND_LOGD:
		{
			usb_comm_measurement_log_enable = 0U;
			usb_cdc_command = USB_CDC_COMMAND_NONE;
			printf("Measurement log disabled\r\n");
			command_result = USB_COMM_OK;
			break;
		}

		case USB_CDC_COMMAND_NONE:
		{
			command_result = USB_COMM_NOT_READY;
			break;
		}

		default:
		{
			usb_cdc_command = USB_CDC_COMMAND_NONE;
			return USB_COMM_COMMAND_ERROR;
		}
	}

	/* process transmit */
	if (USB_CDC_Transmit_Ready() == 0U)
	{
		usb_comm_reset_transmit();
		if (usb_comm_reset_pending != 0U)
		{
			NVIC_SystemReset();
		}
		return command_result;
	}

	if (usb_comm_tx_transmitting != 0U)
	{
		if (USB_CDC_Transmit_Busy() != 0U)
		{
			return command_result;
		}

		usb_comm_tx_transmitting = 0U;
	}

	if (usb_comm_tx_count == 0U)
	{
		if (usb_comm_reset_pending != 0U)
		{
			NVIC_SystemReset();
		}
		return command_result;
	}

	batch_count = 0U;
	batch_read_index = usb_comm_tx_read_index;
	batch_length = 0U;
	while (batch_count < usb_comm_tx_count)
	{
		message_length = usb_comm_tx_length[batch_read_index];
		if (message_length > (USB_COMM_TX_BATCH_SIZE - batch_length))
		{
			break;
		}

		for (uint16_t i = 0U; i < message_length; i++)
		{
			usb_comm_tx_batch[batch_length + i] =
					usb_comm_tx_buffer[batch_read_index][i];
		}
		batch_length += message_length;
		batch_count++;
		batch_read_index++;
		if (batch_read_index >= USB_COMM_TX_BUFFER_COUNT)
		{
			batch_read_index = 0U;
		}
	}

	transmit_result = USB_CDC_Transmit(
			usb_comm_tx_batch,
			batch_length
	);
	if (transmit_result == USB_CDC_OK)
	{
		usb_comm_tx_transmitting = 1U;
		usb_comm_tx_read_index = batch_read_index;
		usb_comm_tx_count -= batch_count;
		return USB_COMM_OK;
	}
	if (
			(transmit_result == USB_CDC_NOT_READY)
			|| (transmit_result == USB_CDC_BUSY)
	)
	{
		return command_result;
	}

	return USB_COMM_TRANSMIT_ERROR;
}

uint8_t usb_comm_measurement_log_enabled(void)
{
	return usb_comm_measurement_log_enable;
}

int usb_comm_write(
		const uint8_t *data,
		uint16_t length
)
{
	if (data == NULL)
	{
		return USB_COMM_PARAM_ERROR;
	}
	if ((length == 0U) || (length > USB_COMM_TX_BUFFER_SIZE))
	{
		return USB_COMM_LENGTH_ERROR;
	}
	if (USB_CDC_Transmit_Ready() == 0U)
	{
		return USB_COMM_NOT_READY;
	}
	if (usb_comm_tx_count >= USB_COMM_TX_BUFFER_COUNT)
	{
		return USB_COMM_BUSY;
	}

	for (uint16_t i = 0U; i < length; i++)
	{
		usb_comm_tx_buffer[usb_comm_tx_write_index][i] = data[i];
	}
	usb_comm_tx_length[usb_comm_tx_write_index] = length;
	usb_comm_tx_write_index++;
	if (usb_comm_tx_write_index >= USB_COMM_TX_BUFFER_COUNT)
	{
		usb_comm_tx_write_index = 0U;
	}
	usb_comm_tx_count++;

	return USB_COMM_OK;
}

static void usb_comm_reset_transmit(void)
{
	usb_comm_tx_read_index = 0U;
	usb_comm_tx_write_index = 0U;
	usb_comm_tx_count = 0U;
	usb_comm_tx_transmitting = 0U;
}
