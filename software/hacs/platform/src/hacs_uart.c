#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_uart.h"
#include "stm32f4xx_hal.h"

#define UART_OP_TIMEOUT_MS   (HAL_MAX_DELAY)

#define BUS_IN_USE  (1)
#define BUS_IDLE    (0)

static UART_HandleTypeDef uart_handles[HACS_NUM_UART_PERIPH];
static DMA_HandleTypeDef uart_rx_dma_handles[HACS_NUM_UART_PERIPH];

static hacs_uart_rx_cb_t uart_rx_ht_cb[HACS_NUM_UART_PERIPH];
static hacs_uart_rx_cb_t uart_rx_tc_cb[HACS_NUM_UART_PERIPH];
static uint32_t uart_rx_size[HACS_NUM_UART_PERIPH];

static uint8_t uart_locks[HACS_NUM_UART_PERIPH];

static void rx_dma_ht(DMA_HandleTypeDef *hdma);
static void rx_dma_tc(DMA_HandleTypeDef *hdma);

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
    hdma->Init.Mode = DMA_CIRCULAR;
    hdma->Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    // set up callbacks
    hdma->XferCpltCallback = rx_dma_tc;
    hdma->XferHalfCpltCallback = rx_dma_ht;

    // remember which bus this DMA stream belongs to
    hdma->Parent = (void *)bus;

    NVIC_SetPriority(hacs_uart_rx_dma_irq[bus],
                     configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    retval = HAL_DMA_Init(hdma);
  }
  // TODO: currently does not support TX DMA

  if (retval == HAL_OK) {
    retval = HAL_UART_Init(p_handle);
  }

  uart_locks[bus] = BUS_IDLE;

  return retval;
}

int hacs_uart_start_listening(hacs_uart_t bus, uint32_t buf, uint32_t size,
                              hacs_uart_rx_cb_t ht_cb,
                              hacs_uart_rx_cb_t tc_cb) {
  int retval;
  DMA_HandleTypeDef *hdma = &uart_rx_dma_handles[bus];
  UART_HandleTypeDef *huart = &uart_handles[bus];

  // check if the bus is already in use
  if (uart_locks[bus] == BUS_IN_USE) {
    return HACS_ERR_ALREADY_IN_USE;
  }
  uart_locks[bus] = BUS_IN_USE;

  uart_rx_ht_cb[bus] = ht_cb;
  uart_rx_tc_cb[bus] = tc_cb;
  uart_rx_size[bus] = size;

  // Must clear interrupt because HAL does not do it for me
  __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
  __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
  __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));

  // Configure the RX DMA
  retval = HAL_DMA_Start_IT(&uart_rx_dma_handles[bus],
                            (uint32_t)&huart->Instance->DR,
                            buf, size);

  NVIC_EnableIRQ(hacs_uart_rx_dma_irq[bus]);

  // Configure DMA on USART side
  __HAL_UART_CLEAR_OREFLAG(huart); // clear overrun flag
  huart->Instance->CR3 |= USART_CR3_DMAR;

  return retval;
}

int hacs_uart_stop_listening(hacs_uart_t bus) {
  DMA_HandleTypeDef *hdma = &uart_rx_dma_handles[bus];
  UART_HandleTypeDef *huart = &uart_handles[bus];

  uart_locks[bus] = BUS_IDLE;

  huart->Instance->CR3 &= ~(USART_CR3_DMAR);

  NVIC_DisableIRQ(hacs_uart_rx_dma_irq[bus]);

  return HAL_DMA_Abort(hdma);
}

int hacs_uart_blocking_transmit(hacs_uart_t bus, uint8_t *wbuf, size_t wsize) {
  return -1;
}

int hacs_uart_blocking_receive(hacs_uart_t bus, uint8_t *rbuf, size_t rsize) {
  return -1;
}

static void rx_dma_ht(DMA_HandleTypeDef *hdma) {
  // Figure out the bus number
  hacs_uart_t bus = (hacs_uart_t)hdma->Parent;

  // Invoke callback, providing the actual length read
  uart_rx_ht_cb[bus](uart_rx_size[bus] - __HAL_DMA_GET_COUNTER(hdma));
}

static void rx_dma_tc(DMA_HandleTypeDef *hdma) {
  // Figure out the bus number
  hacs_uart_t bus = (hacs_uart_t)hdma->Parent;

  // Invoke callback, providing the actual length read
  uart_rx_tc_cb[bus](uart_rx_size[bus] - __HAL_DMA_GET_COUNTER(hdma));
}

// NOTE: the bus-to-dma mapping is hard-coded here!!!
void DMA2_Stream1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&uart_rx_dma_handles[HACS_UART_MPU6050]);
}

// NOTE: the bus-to-dma mapping is hard-coded here!!!
void DMA2_Stream2_IRQHandler(void) {
  HAL_DMA_IRQHandler(&uart_rx_dma_handles[HACS_UART_GPS]);
}
