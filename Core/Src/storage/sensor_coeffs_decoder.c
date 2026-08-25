#include "storage/sensor_coeffs_decoder.h"

#define SENSOR_COEFFS_VERSION			1U
#define SENSOR_COEFFS_HEADER_SIZE		8U
#define SENSOR_COEFFS_CRC_SIZE			4U
#define SENSOR_COEFFS_SEGMENT_HEADER_SIZE	20U
#define SENSOR_COEFFS_FLOAT_SIZE			4U

static uint16_t Sensor_Coeffs_Read_16_Bit(const uint8_t *data)
{
	return (
			(uint16_t)data[0]
			| ((uint16_t)data[1] << 8)
	);
}

static uint32_t Sensor_Coeffs_Read_32_Bit(const uint8_t *data)
{
	return (
			(uint32_t)data[0]
			| ((uint32_t)data[1] << 8)
			| ((uint32_t)data[2] << 16)
			| ((uint32_t)data[3] << 24)
	);
}

static float Sensor_Coeffs_Read_Float(const uint8_t *data)
{
	union
	{
		uint32_t uint32_value;
		float float_value;
	} value;

	value.uint32_value = Sensor_Coeffs_Read_32_Bit(data);

	return value.float_value;
}

static int Sensor_Coeffs_Check_Length(
		uint32_t data_length,
		uint32_t read_position,
		uint32_t required_length
)
{
	if (read_position > data_length)
	{
		return 0;
	}

	return required_length <= (data_length - read_position);
}

static uint32_t Sensor_Coeffs_CRC32(const uint8_t *data, uint32_t length)
{
	uint32_t crc;

	crc = 0xFFFFFFFFU;

	for (uint32_t i = 0; i < length; i++)
	{
		crc ^= data[i];

		for (uint8_t bit = 0; bit < 8U; bit++)
		{
			if ((crc & 0x00000001U) != 0U)
			{
				crc = (crc >> 1) ^ 0xEDB88320U;
			}
			else
			{
				crc >>= 1;
			}
		}
	}

	return crc ^ 0xFFFFFFFFU;
}

int Sensor_Coeffs_Decode(
		const uint8_t *data,
		uint32_t length,
		Curve_t *curve
)
{
	Curve_t decoded_curve = {0};
	uint32_t data_length;
	uint32_t read_position;
	uint32_t stored_crc;
	uint32_t calculated_crc;
	uint16_t version;
	uint16_t segment_count;

	if ((data == NULL) || (curve == NULL))
	{
		return SENSOR_COEFFS_DECODE_PARAM_ERROR;
	}

	if (length < (SENSOR_COEFFS_HEADER_SIZE + SENSOR_COEFFS_CRC_SIZE))
	{
		return SENSOR_COEFFS_DECODE_FORMAT_ERROR;
	}
	data_length = length - SENSOR_COEFFS_CRC_SIZE;

	if (
			(data[0] != 'S') || (data[1] != 'C')
			|| (data[2] != 'V') || (data[3] != '1')
	)
	{
		return SENSOR_COEFFS_DECODE_FORMAT_ERROR;
	}

	version = Sensor_Coeffs_Read_16_Bit(&data[4]);
	if (version != SENSOR_COEFFS_VERSION)
	{
		return SENSOR_COEFFS_DECODE_VERSION_ERROR;
	}

	segment_count = Sensor_Coeffs_Read_16_Bit(&data[6]);
	if ((segment_count == 0U) || (segment_count > SENSOR_FIT_MAX_SEGMENTS))
	{
		return SENSOR_COEFFS_DECODE_FORMAT_ERROR;
	}

	stored_crc = Sensor_Coeffs_Read_32_Bit(&data[data_length]);
	calculated_crc = Sensor_Coeffs_CRC32(data, data_length);
	if (stored_crc != calculated_crc)
	{
		return SENSOR_COEFFS_DECODE_CRC_ERROR;
	}

	read_position = SENSOR_COEFFS_HEADER_SIZE;
	decoded_curve.segment_count = segment_count;

	for (uint16_t i = 0; i < segment_count; i++)
	{
		CurveSegment_t *segment;
		uint32_t order;
		uint32_t coefficient_size;

		if (
				Sensor_Coeffs_Check_Length(
						data_length, read_position,
						SENSOR_COEFFS_SEGMENT_HEADER_SIZE
				) == 0
		)
		{
			return SENSOR_COEFFS_DECODE_FORMAT_ERROR;
		}

		segment = &decoded_curve.segments[i];
		segment->z_min = Sensor_Coeffs_Read_Float(&data[read_position]);
		read_position += SENSOR_COEFFS_FLOAT_SIZE;
		segment->z_max = Sensor_Coeffs_Read_Float(&data[read_position]);
		read_position += SENSOR_COEFFS_FLOAT_SIZE;
		segment->r_left = Sensor_Coeffs_Read_Float(&data[read_position]);
		read_position += SENSOR_COEFFS_FLOAT_SIZE;
		segment->r_right = Sensor_Coeffs_Read_Float(&data[read_position]);
		read_position += SENSOR_COEFFS_FLOAT_SIZE;
		order = Sensor_Coeffs_Read_32_Bit(&data[read_position]);
		read_position += sizeof(order);

		if (order > SENSOR_FIT_MAX_ORDER)
		{
			return SENSOR_COEFFS_DECODE_FORMAT_ERROR;
		}

		coefficient_size = (order + 1U) * SENSOR_COEFFS_FLOAT_SIZE;
		if (
				Sensor_Coeffs_Check_Length(
						data_length, read_position, coefficient_size
				) == 0
		)
		{
			return SENSOR_COEFFS_DECODE_FORMAT_ERROR;
		}

		for (uint32_t coefficient = 0; coefficient <= order; coefficient++)
		{
			segment->coeffs[coefficient] = Sensor_Coeffs_Read_Float(
					&data[read_position]
			);
			read_position += SENSOR_COEFFS_FLOAT_SIZE;
		}

		segment->order = (int)order;
	}

	if (read_position != data_length)
	{
		return SENSOR_COEFFS_DECODE_FORMAT_ERROR;
	}

	*curve = decoded_curve;

	return SENSOR_COEFFS_DECODE_OK;
}
