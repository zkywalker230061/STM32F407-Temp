#include "communication/sensor_coeffs_transfer.h"

#define SENSOR_COEFFS_TRANSFER_HEADER_SIZE		12U
#define SENSOR_COEFFS_TRANSFER_FRAME_MAX_SIZE	360U

#define SENSOR_COEFFS_TRANSFER_RECEIVING		0U
#define SENSOR_COEFFS_TRANSFER_READY			1U
#define SENSOR_COEFFS_TRANSFER_ERROR			2U

static uint8_t sensor_coeffs_transfer_buffer[
		SENSOR_COEFFS_TRANSFER_FRAME_MAX_SIZE
];
static uint32_t sensor_coeffs_received_length;
static volatile uint32_t sensor_coeffs_binary_length;
static volatile uint8_t sensor_coeffs_transfer_state;
static volatile int sensor_coeffs_transfer_error;

static uint32_t Sensor_Coeffs_Transfer_Read_32_Bit(const uint8_t *data)
{
	return (
			(uint32_t)data[0]
			| ((uint32_t)data[1] << 8)
			| ((uint32_t)data[2] << 16)
			| ((uint32_t)data[3] << 24)
	);
}

int Sensor_Coeffs_Transfer_Receive(
		const uint8_t *data,
		uint32_t length
)
{
	uint32_t remaining_length;
	uint32_t expected_length;

	if (data == NULL)
	{
		return SENSOR_COEFFS_TRANSFER_PARAM_ERROR;
	}

	if (sensor_coeffs_transfer_state != SENSOR_COEFFS_TRANSFER_RECEIVING)
	{
		return SENSOR_COEFFS_TRANSFER_STATE_ERROR;
	}

	remaining_length = (
			SENSOR_COEFFS_TRANSFER_FRAME_MAX_SIZE
			- sensor_coeffs_received_length
	);
	if (length > remaining_length)
	{
		sensor_coeffs_transfer_error = SENSOR_COEFFS_TRANSFER_LENGTH_ERROR;
		sensor_coeffs_transfer_state = SENSOR_COEFFS_TRANSFER_ERROR;
		return SENSOR_COEFFS_TRANSFER_LENGTH_ERROR;
	}

	for (uint32_t i = 0; i < length; i++)
	{
		sensor_coeffs_transfer_buffer[sensor_coeffs_received_length + i] = data[i];
	}
	sensor_coeffs_received_length += length;

	if (sensor_coeffs_received_length < SENSOR_COEFFS_TRANSFER_HEADER_SIZE)
	{
		return SENSOR_COEFFS_TRANSFER_NOT_READY;
	}

	if (
			(sensor_coeffs_transfer_buffer[0] != 'S')
			|| (sensor_coeffs_transfer_buffer[1] != 'C')
			|| (sensor_coeffs_transfer_buffer[2] != 'U')
			|| (sensor_coeffs_transfer_buffer[3] != 'P')
			|| (sensor_coeffs_transfer_buffer[4] < 1U)
			|| (sensor_coeffs_transfer_buffer[4] > 2U)
			|| (sensor_coeffs_transfer_buffer[5] > 3U)
			|| (sensor_coeffs_transfer_buffer[6] != 0U)
			|| (sensor_coeffs_transfer_buffer[7] != 0U)
	)
	{
		sensor_coeffs_transfer_error = SENSOR_COEFFS_TRANSFER_FORMAT_ERROR;
		sensor_coeffs_transfer_state = SENSOR_COEFFS_TRANSFER_ERROR;
		return SENSOR_COEFFS_TRANSFER_FORMAT_ERROR;
	}

	sensor_coeffs_binary_length = Sensor_Coeffs_Transfer_Read_32_Bit(
			&sensor_coeffs_transfer_buffer[8]
	);
	if (
			(sensor_coeffs_binary_length < SENSOR_COEFFS_BINARY_MIN_SIZE)
			|| (sensor_coeffs_binary_length > SENSOR_COEFFS_BINARY_MAX_SIZE)
	)
	{
		sensor_coeffs_transfer_error = SENSOR_COEFFS_TRANSFER_LENGTH_ERROR;
		sensor_coeffs_transfer_state = SENSOR_COEFFS_TRANSFER_ERROR;
		return SENSOR_COEFFS_TRANSFER_LENGTH_ERROR;
	}

	expected_length = (
			SENSOR_COEFFS_TRANSFER_HEADER_SIZE
			+ sensor_coeffs_binary_length
	);
	if (sensor_coeffs_received_length > expected_length)
	{
		sensor_coeffs_transfer_error = SENSOR_COEFFS_TRANSFER_LENGTH_ERROR;
		sensor_coeffs_transfer_state = SENSOR_COEFFS_TRANSFER_ERROR;
		return SENSOR_COEFFS_TRANSFER_LENGTH_ERROR;
	}

	if (sensor_coeffs_received_length < expected_length)
	{
		return SENSOR_COEFFS_TRANSFER_NOT_READY;
	}

	sensor_coeffs_transfer_state = SENSOR_COEFFS_TRANSFER_READY;

	return SENSOR_COEFFS_TRANSFER_OK;
}

int Sensor_Coeffs_Transfer_Get_Data(
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
		return SENSOR_COEFFS_TRANSFER_PARAM_ERROR;
	}

	*adc_device_id = 0U;
	*channel = 0xFFU;
	*binary_data = NULL;
	*binary_length = 0U;

	if (sensor_coeffs_transfer_state == SENSOR_COEFFS_TRANSFER_ERROR)
	{
		return sensor_coeffs_transfer_error;
	}

	if (sensor_coeffs_transfer_state != SENSOR_COEFFS_TRANSFER_READY)
	{
		return SENSOR_COEFFS_TRANSFER_NOT_READY;
	}

	*adc_device_id = sensor_coeffs_transfer_buffer[4];
	*channel = sensor_coeffs_transfer_buffer[5];
	*binary_data = &sensor_coeffs_transfer_buffer[
			SENSOR_COEFFS_TRANSFER_HEADER_SIZE
	];
	*binary_length = sensor_coeffs_binary_length;

	return SENSOR_COEFFS_TRANSFER_OK;
}

void Sensor_Coeffs_Transfer_Reset(void)
{
	sensor_coeffs_received_length = 0U;
	sensor_coeffs_binary_length = 0U;
	sensor_coeffs_transfer_error = SENSOR_COEFFS_TRANSFER_OK;
	sensor_coeffs_transfer_state = SENSOR_COEFFS_TRANSFER_RECEIVING;
}

