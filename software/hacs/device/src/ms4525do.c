#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_i2c_master.h"
#include "hacs_pstore.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ms4525do.h"

#define STATUS_BIT_MASK   (0xA0)
#define STATUS_NORMAL     (0x00)
#define STATUS_STALE      (0x80)
#define STATUS_FAULT      (0xA0)

#define RAW_COUNT_MIN     (0x0666)
#define RAW_COUNT_MAX     (0x399A)

#define P_MAX_PA          (6894.757f)
#define P_MIN_PA          (-6894.757f)

#define ZERO_CAL_ITERATIONS           (5)
#define ZERO_CAL_MEASUREMENT_DELAY_MS (40)

static xSemaphoreHandle dev_lock;
static float zero_offset;

int ms4525do_early_init(void)
{
  dev_lock = xSemaphoreCreateRecursiveMutex();

  // Try to read offset from PSTORE
  if (hacs_pstore_get(HACS_PSTORE_AIRSPEED_ZERO_CAL, (uint8_t*)&zero_offset, sizeof(zero_offset)) != HACS_NO_ERROR) {
    zero_offset = 0.0;
  }
  printf("airspeed cal: %f\r\n", zero_offset);

  // Init the INT pin here if we were to use them in the future.
  return HACS_NO_ERROR;
}

// Get differential pressure reading in Pa. Blocks until completion.
int ms4525do_get_dp(float *p_result)
{
  uint8_t buf[2];
  int retval;

  xSemaphoreTakeRecursive(dev_lock, portMAX_DELAY);

  // Because the MS4525DO I got has "Standard Configuration", meaning
  // that it will continuously sample at max freequency on its own, I
  // can directly read the result at any time (given that our read
  // frequency is lower than the automatic sampling frequency)
  retval = i2c_master_receive(HACS_I2C, MS4525DO_I2C_ADDRESS, buf, sizeof(buf));
  HACS_REQUIRES(retval >= 0, done);

  // check the status bit
  if ((buf[0] & STATUS_BIT_MASK) == STATUS_NORMAL) {
    uint16_t raw = ((uint16_t)(buf[0] & ~STATUS_BIT_MASK) << 8) | (uint16_t)buf[1];

    // Refer to the official MS4525DO datasheet for the below calculation (specific to MS4525DO-DS5AI001P)
    *p_result = ((float)raw - 0.1f * 16383) * (P_MAX_PA - P_MIN_PA) / (0.8f * 16383) + P_MIN_PA;
  }

done:
  xSemaphoreGiveRecursive(dev_lock);
  return retval;
}

int ms4525do_get_dp_calibrated(float *p_result)
{
  int retval;
  float temp_res;

  retval = ms4525do_get_dp(&temp_res);
  HACS_REQUIRES(retval >= 0, done);

  // eliminate zero offset
  *p_result = temp_res - zero_offset;

done:
  return retval;
}

// Perform calibration to eliminate zero offset
int ms4525do_zero_cal(void)
{
  float result;
  int retval;

  xSemaphoreTakeRecursive(dev_lock, portMAX_DELAY);

  zero_offset = 0;
  for (int i = 0; i < ZERO_CAL_ITERATIONS; i++) {
    vTaskDelay(MS_TO_TICKS(ZERO_CAL_MEASUREMENT_DELAY_MS));
    retval = ms4525do_get_dp(&result);
    HACS_REQUIRES(retval >= 0, done);
    zero_offset += result;
  }
  zero_offset = zero_offset / (float)ZERO_CAL_ITERATIONS;

  // Save cal result to PSTORE
  retval = hacs_pstore_set(HACS_PSTORE_AIRSPEED_ZERO_CAL, (uint8_t*)&zero_offset, sizeof(zero_offset));

done:
  xSemaphoreGiveRecursive(dev_lock);
  return retval;
}
