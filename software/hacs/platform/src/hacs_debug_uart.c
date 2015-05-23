#include "hacs_platform.h"
#include "hacs_debug_uart.h"

#include "stm32f4xx_hal.h"
#include "hacs_platform_resources.h"

static UART_HandleTypeDef UartHandle; 

int debug_uart_init(uint32_t baud)
{
	UartHandle.Instance        = USART2;
	UartHandle.Init.BaudRate   = baud;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;

	return HAL_UART_Init(&UartHandle);
}

int debug_uart_putchar(char c)
{
	return HAL_UART_Transmit(&UartHandle, (uint8_t *)&c, 1, 0xFFFF);
}

char debug_uart_getchar(void)
{
	char c;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&c, 1, 0xFFFF);
  return c;
}

int debug_uart_inpstat(void)
{
	return (USART2->SR & USART_SR_RXNE);
}
