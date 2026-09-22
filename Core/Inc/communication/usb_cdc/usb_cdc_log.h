#ifndef USB_CDC_LOG_H
#define USB_CDC_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

int _write(
		int file,
		char *data,
		int length
);

#ifdef __cplusplus
}
#endif

#endif  /* USB_CDC_LOG_H */
