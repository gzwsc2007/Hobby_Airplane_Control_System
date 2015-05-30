#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_uart.h"
#include "stm32f4xx_hal.h"

#define UART_OP_TIMEOUT_MS   (HAL_MAX_DELAY)

static UART_HandleTypeDef uart_handles[HACS_NUM_UART_PERIPH];
static DMA_HandleTypeDef uart_rx_dma_handles[HACS_NUM_UART_PERIPH];
static hacs_uart_rx_cb_t uart_rx_cb[HACS_NUM_UART_PERIPH];
static uint32_t uart_rx_size[HACS_NUM_UART_PERIPH];

int hacs_uart_init(hacs_uart_t bus, uint32_t baud, uint8_t use_tx_dma, uint8_t use_rx_dma) {
  UART_HandleTypeDef *p_handle = &uart_handles[bus];
  int retval;

  p_handle->Instance        = hacs_uart_instances[bus];
  p_handle->Init.BaudRate   = baud;
  p_handle->Init.WordLength = UART_WORDLENGTH_8B;
  p_handle->Init.StopBits   = UART_STOPBITS_1;
  p_handle->Init.Parity     = UART_PARITY_NONE;
  p_handle->Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  p_handle->Init.Mode       = UART_MODE_TX_RX;

  // Initialize DMA
  if (use_rx_dma) {
    DMA_HandleTypeDef *hdma = &uart_rx_dma_handles[bus];
    hdma->Instance = hacs_uart_rx_dma_stream[bus];
    hdma->Init.Channel = hacs_uart_rx_dma_chan[bus];
    hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma->Init.MemInc = DMA_MINC_ENABLE;
    hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma->Init.Mode = DMA_NORMAL;
    hdma->Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    retval = HAL_DMA_Init(hdma);
  }
  // TODO: currently does not support TX DMA

  if (retval == HAL_OK) {
    retval = HAL_UART_Init(p_handle);
  }

  return retval;
}

int hacs_uart_start_listening(hacs_uart_t bus, uint32_t buf, uint32_t size, 
                              hacs_uart_rx_cb_t cb) {
  int retval;

  uart_rx_cb[bus] = cb;
  uart_rx_size[bus] = size;

  // Configure the RX DMA
  retval = HAL_DMA_Start(&uart_rx_dma_handles[bus], 
                         (uint32_t)&hacs_uart_instances[bus]->DR, 
                         buf, size);

  // Use the IDLE interrupt to synchronize RX
  __HAL_UART_ENABLE_IT(&uart_handles[bus], UART_IT_IDLE);

  return retval;
}

int hacs_uart_stop_listening(hacs_uart_t bus) {
  DMA_HandleTypeDef *hdma = &uart_rx_dma_handles[bus];

  __HAL_DMA_DISABLE(hdma);
  __HAL_UART_DISABLE_IT(&uart_handles[bus], UART_IT_IDLE);

  return 0;
}

int hacs_uart_blocking_transmit(hacs_uart_t bus, uint8_t *wbuf, size_t wsize) {
  return -1;
}

int hacs_uart_blocking_receive(hacs_uart_t bus, uint8_t *rbuf, size_t rsize) {
  return -1;
}

static void uart_irq_handler(hacs_uart_t bus) {
  UART_HandleTypeDef *huart = &uart_handles[bus];

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    // Disable DMA
    DMA_HandleTypeDef *hdma = &uart_rx_dma_handles[bus];
    __HAL_DMA_DISABLE(hdma);
    huart->Instance->CR3 &= ~(USART_CR3_DMAR); // disable DMA on UART side

    // Invoke callback, providing the actual length read
    uart_rx_cb[bus](uart_rx_size[bus] - __HAL_DMA_GET_COUNTER(hdma));

    // Enable DMA again
    __HAL_DMA_SET_COUNTER(hdma, uart_rx_size[bus]);
    __HAL_DMA_ENABLE(hdma);
    huart->Instance->CR3 |= USART_CR3_DMAR; // enable DMA on UART side
  }
}

void USART1_IRQHandler(void) {
  uart_irq_handler(HACS_UART_GPS);
}

void USART6_IRQHandler(void) {
  uart_irq_handler(HACS_UART_MPU6050);
}
