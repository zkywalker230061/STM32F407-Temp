#ifndef USB_CDC_SCUP_H
#define USB_CDC_SCUP_H

#include <stdint.h>

#include "common/error_code.h"


#ifdef __cplusplus
extern "C" {
#endif

ErrorCode_t USB_CDC_SCUP_Receive(
		const uint8_t *data,
		uint32_t length
);

ErrorCode_t USB_CDC_SCUP_Get_Data(
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
