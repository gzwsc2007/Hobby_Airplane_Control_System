#include "hacs_platform.h"
#include "hacs_debug_uart.h"
#include "stm32f4xx_hal.h"
#include "hacs_platform_resources.h"
#include "semphr.h"

static UART_HandleTypeDef UartHandle;
static xSemaphoreHandle rx_sem;
static volatile char buffered_char;

int debug_uart_init(uint32_t baud)
{
	rx_sem = xSemaphoreCreateBinary();

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
	while(!(USART2->SR & USART_SR_TXE));
	USART2->DR = c;
	while(!(USART2->SR & USART_SR_TC));
	return 0;
}

char debug_uart_blocking_getchar(void)
{
	// This is the only place where the UART control register is being modified.
	// A critical section will be enough
	hacs_enter_critical();
	__HAL_UART_CLEAR_OREFLAG(&UartHandle);
	hacs_exit_critical();

	xSemaphoreTake(rx_sem, portMAX_DELAY);
	return buffered_char;
}

int debug_uart_wait_rxne(void)
{
	debug_uart_blocking_getchar();
	return 1;
}

// The only IRQ enabled is RXNE
void USART2_IRQHandler(void)
{
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	buffered_char = USART2->DR;
	xSemaphoreGiveFromISR(rx_sem, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
