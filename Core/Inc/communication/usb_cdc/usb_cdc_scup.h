#ifndef USB_CDC_SCUP_H
#define USB_CDC_SCUP_H

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#define USB_CDC_SCUP_OK            0
#define USB_CDC_SCUP_NOT_READY     1
#define USB_CDC_SCUP_PARAM_ERROR  -1
#define USB_CDC_SCUP_FORMAT_ERROR -2
#define USB_CDC_SCUP_LENGTH_ERROR -3
#define USB_CDC_SCUP_STATE_ERROR  -4


int USB_CDC_SCUP_Receive(
		const uint8_t *data,
		uint32_t length
);

int USB_CDC_SCUP_Get_Data(
		uint8_t *adc_device_id,
		uint8_t *channel,
		const uint8_t **binary_data,
		uint32_t *binary_length
);

void USB_CDC_SCUP_Reset(void);

#ifdef __cplusplus
}
#endif

#endif  /* USB_CDC_SCUP_H */
