#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_spi_master.h"
#include "stm32f4xx_hal.h"
#include "hacs_gpio.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define SPI_OP_TIMEOUT_MS   (100)

static SPI_HandleTypeDef spi_handles[HACS_NUM_SPI_PERIPH];
static xSemaphoreHandle spi_locks[HACS_NUM_SPI_PERIPH];

static uint32_t calc_prescaler_from_freq(hacs_spi_t bus, uint32_t freq)
{
  SPI_TypeDef* spi = hacs_spi_instances[bus];
  uint32_t pclk;
  uint32_t ratio;
  uint8_t presc = 0;

  if (spi == SPI2 || spi == SPI3) { // on APB1
    pclk = HAL_RCC_GetPCLK1Freq();
  } else { // on APB2
    pclk = HAL_RCC_GetPCLK2Freq();
  }

  ratio = pclk / freq;

  do {
    presc++;
    ratio = ratio >> 1;
  } while (ratio > 0);

  if (presc > 8) presc = 8;

  return (presc - 1) & 0x7;
}

/* Current implementation defines hacs_spi_t as int, which can be used as an index
* to the look up tables */

int spi_master_init(hacs_spi_t bus, uint32_t freq, uint8_t cpol, uint8_t cpha)
{
  SPI_HandleTypeDef *p_handle = &spi_handles[bus];

  // Initialize the CS pin
  gpio_init_pin(hacs_spi_cs_port[bus], hacs_spi_cs_pin[bus], HACS_GPIO_MODE_OUTPUT_PP,
                HACS_GPIO_NO_PULL);
  gpio_write_low(hacs_spi_cs_port[bus], hacs_spi_cs_pin[bus]);

  p_handle->Instance = hacs_spi_instances[bus];
  p_handle->Init.Mode = SPI_MODE_MASTER;
  p_handle->Init.Direction = SPI_DIRECTION_2LINES;
  p_handle->Init.DataSize = SPI_DATASIZE_8BIT;

  if (cpol == HACS_SPI_CPOL_1) {
    p_handle->Init.CLKPolarity = SPI_POLARITY_HIGH; // CPOL = 1
  } else if (cpol == HACS_SPI_CPOL_0) {
    p_handle->Init.CLKPolarity = SPI_POLARITY_LOW; // CPOL = 0
  } else {
    return -1; // invalid param
  }

  if (cpha == HACS_SPI_CPHA_1) {
    p_handle->Init.CLKPhase = SPI_PHASE_2EDGE;   // CPHA = 1
  } else if (cpha == HACS_SPI_CPHA_0) {
    p_handle->Init.CLKPhase = SPI_PHASE_1EDGE;   // CPHA = 0
  } else {
    return -1; // invalid param
  }

  p_handle->Init.NSS = SPI_NSS_SOFT;
  p_handle->Init.BaudRatePrescaler = calc_prescaler_from_freq(bus, freq) << 3;
  p_handle->Init.FirstBit = SPI_FIRSTBIT_MSB;
  p_handle->Init.TIMode = SPI_TIMODE_DISABLED;
  p_handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  p_handle->Init.CRCPolynomial = 7; // don't care

  // Create bus lock
  spi_locks[bus] = xSemaphoreCreateMutex();

  return HAL_SPI_Init(p_handle);
}

int spi_master_exchange(hacs_spi_t bus, uint8_t *wbuf, uint8_t *rbuf, size_t size) {
  return HAL_SPI_TransmitReceive(&spi_handles[bus], wbuf, rbuf, size, SPI_OP_TIMEOUT_MS);
}

int spi_master_transfer(hacs_spi_t bus, uint8_t *wbuf, size_t wsize, uint8_t *rbuf, size_t rsize)
{
  HAL_SPI_Transmit(&spi_handles[bus], wbuf, wsize, SPI_OP_TIMEOUT_MS);
  HAL_SPI_Receive(&spi_handles[bus], rbuf, rsize, SPI_OP_TIMEOUT_MS);
  return 0;
}

int spi_master_write(hacs_spi_t bus, uint8_t *wbuf, size_t wsize)
{
  return HAL_SPI_Transmit(&spi_handles[bus], wbuf, wsize, SPI_OP_TIMEOUT_MS);
}

int spi_master_read(hacs_spi_t bus, uint8_t *rbuf, size_t rsize)
{
  return HAL_SPI_Receive(&spi_handles[bus], rbuf, rsize, SPI_OP_TIMEOUT_MS);
}

void spi_master_assert_cs(hacs_spi_t bus) {
  xSemaphoreTake(spi_locks[bus], portMAX_DELAY);
  gpio_write_low(hacs_spi_cs_port[bus], hacs_spi_cs_pin[bus]);
}

void spi_master_deassert_cs(hacs_spi_t bus) {
  gpio_write_high(hacs_spi_cs_port[bus], hacs_spi_cs_pin[bus]);
  xSemaphoreGive(spi_locks[bus]);
}
