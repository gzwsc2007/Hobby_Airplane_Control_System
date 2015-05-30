#ifndef _HACS_TELEMETRY_H_
#define _HACS_TELEMETRY_H_

typedef enum {
	TELEM_TX_PFD = 1,
	TELEM_TX_NAVD,
	TELEM_TX_SYSID,
} tx_type_t;

#define HACS_TELEM_TX_QUEUE_LEN  5

void hacs_telemetry_rx_task(void *param);
void hacs_telemetry_tx_task(void *param);

#endif
