#ifndef _HACS_ERROR_CODES_H_
#define _HACS_ERROR_CODES_H_

typedef enum {
	HACS_NO_ERROR = 0,

	HACS_NRF24_TX_FAILED = -10,
	HACS_NRF24_RX_INVALID_LENGTH,
	HACS_NRF24_RX_NOT_READY,

	HACS_EXTI_OVERWRITE_WARN = -20,

} hacs_error_t;

#endif
