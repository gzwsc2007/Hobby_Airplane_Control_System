#include "hacs_platform.h"
#include "stm32f4xx_hal.h"

static CRC_HandleTypeDef hcrc;

int hacs_crc_init(void)
{
  hcrc.Instance = CRC;
  __CRC_CLK_ENABLE();
  return HAL_CRC_Init(&hcrc);
}

uint32_t hacs_crc32_calc(uint8_t *buf, uint32_t len)
{
  uint32_t crc;
  uint32_t remain;
  uint32_t rest = 0;

  // Cortex M4 supports 32-bit unaligned accesses
  crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)buf, len/4);

  // Deal with the rest. Use zero padding at the end.
  remain = len % 4;
  if (remain != 0) {
    while (remain > 0) {
      remain--;
      rest |= (uint32_t)(*(buf + len - 1 - remain)) << (remain * 8);
    }
    crc = HAL_CRC_Accumulate(&hcrc, &rest, 1);
  }

  return crc;
}

uint32_t hacs_crc32_accum(uint8_t *buf, uint32_t len)
{
  // Cortex M4 supports 32-bit unaligned accesses
  return -1;
}
