#ifndef _HACS_DEBUG_UART_H_
#define _HACS_DEBUG_UART_H_

int debug_uart_init(uint32_t baud);
int debug_uart_putchar(char c);
char debug_uart_blocking_getchar(void);
int debug_uart_rxne(void);

#endif
