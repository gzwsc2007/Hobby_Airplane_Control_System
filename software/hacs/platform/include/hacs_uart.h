#ifndef _HACS_UART_H_
#define _HACS_UART_H_

#include <stdlib.h>
#include "hacs_platform_resources.h"

#define HACS_UART_USE_RX_DMA      1
#define HACS_UART_NOT_USE_RX_DMA  0
#define HACS_UART_USE_TX_DMA      1
#define HACS_UART_NOT_USE_TX_DMA  0

typedef void (*hacs_uart_rx_cb_t)(uint32_t rx_len);

int hacs_uart_init(hacs_uart_t bus, uint32_t baud, uint8_t use_tx_dma, uint8_t use_rx_dma);

int hacs_uart_start_listening(hacs_uart_t bus, uint32_t buf, uint32_t size,
                              hacs_uart_rx_cb_t ht_cb,
                              hacs_uart_rx_cb_t tc_cb);

int hacs_uart_stop_listening(hacs_uart_t bus);

int hacs_uart_blocking_transmit(hacs_uart_t bus, uint8_t *wbuf, size_t wsize);

int hacs_uart_blocking_receive(hacs_uart_t bus, uint8_t *rbuf, size_t rsize);

#endif