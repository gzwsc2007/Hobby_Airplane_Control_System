#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_debug_uart.h"
#include "hacs_console.h"
#include "hacs_console_commands.h"
#include "FreeRTOS.h"
#include "task.h"

#define LF  0x0A
#define CR  0x0D
#define BS  0x08
#define DEL 0x7F

static void console_put_eol(void)
{
	debug_uart_putchar(CR);
	debug_uart_putchar(LF);
}

static void console_put_bs(void)
{
	debug_uart_putchar(BS);
	debug_uart_putchar(' ');
	debug_uart_putchar(BS);
}

void hacs_console_task(void *param)
{
	char buf[CONSOLE_LINE_BUFFER_SIZE];
	uint8_t cursor = 0;
	char c;
	int retval = 0;

	printf("Welcome to HACS console!\n\r");
	debug_uart_putchar('>');

	while(1) {
		while(debug_uart_inpstat()) {
			c = debug_uart_getchar();

			if (c == LF || c == CR) {
				console_put_eol();
				buf[cursor] = '\0';
				cursor = 0;
				retval = hacs_console_cmd_dispatch(buf);
				if (retval != 0) {
					printf("*** Status %d ***", retval);
				}
				console_put_eol();
				debug_uart_putchar('>');
			} else if (c == BS || c == DEL) {
				if (cursor > 0) {
					console_put_bs();
					cursor--;
				}
			} else {
				// Make sure we don't overflow the line buffer.
				// Need to reserve 1 byte for the NULL terminator.
				if (cursor >= CONSOLE_LINE_BUFFER_SIZE - 1) {
					cursor = CONSOLE_LINE_BUFFER_SIZE - 2;
					debug_uart_putchar(BS);
				}
				debug_uart_putchar(c);
				buf[cursor] = c;
				cursor++;
			}
		}

		// TODO: change this to wait on an event (instead of contantly polling)
		vTaskDelay(MS_TO_TICKS(40));
	}
}
