#include <math.h>
#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_i2c_master.h"
#include "bmp085.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define BMP085_DRIVER_STACK_SIZE  (128)
#define BMP085_DRIVER_PRIORITY    (6)

#define SAMPLE_IN_PROGRESS    (1)
#define READY_FOR_SAMPLE      (0)

#define BMP085_TEMPERATURE_CONVERSION_MS      (5)
#define PRESSURE_REF_CAL_ITERATIONS           (3)

static volatile uint8_t sample_lock;
static xSemaphoreHandle start_sema4;
static bmp085_cb_t singleshot_done_cb;
static float *p_singleshot_alt;
static int16_t *p_singleshot_temp; // in 0.1 Celcius

// calibration values
static int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
static uint16_t ac4, ac5, ac6;

// reference pressure at ground level. Calculated when the
// device is first initialized
static uint32_t press_ref;

static int bmp085_late_init(void);
static int bmp085_get_temperature(int16_t *p_temp, int32_t *b5);
static int bmp085_get_pressure(int32_t b5, int32_t *p_pres);
static int bmp085_get_temp_and_press(int16_t *p_temp, int32_t *p_pres);
static float pressure_to_altitude(int32_t pres);

static void bmp085_driver_task(void *param) {
  int32_t pressure;
  int retval;

  retval = bmp085_late_init();
  HACS_REQUIRES(retval == HACS_NO_ERROR, error);

  // Obtain pressure at ground-level for reference
  press_ref = 0;
  for (int i = 0; i < PRESSURE_REF_CAL_ITERATIONS; i++) {
    retval = bmp085_get_temp_and_press(NULL, &pressure);
    HACS_REQUIRES(retval == HACS_NO_ERROR, error);
    press_ref += pressure;
  }
  press_ref = press_ref / PRESSURE_REF_CAL_ITERATIONS;

  while (1) {
    // Wait to be notified
    sample_lock = READY_FOR_SAMPLE;
    xSemaphoreTake(start_sema4, portMAX_DELAY);

    retval = bmp085_get_temp_and_press(p_singleshot_temp, &pressure);
    HACS_REQUIRES(retval == HACS_NO_ERROR, sample_done);

    *p_singleshot_alt = pressure_to_altitude(pressure);

sample_done:
    if (singleshot_done_cb) singleshot_done_cb(retval);
  }

error:
  printf("Error in bmp085 driver task: %d\r\n", retval);
  vTaskSuspend(NULL);
}

int bmp085_early_init(void) {
  sample_lock = SAMPLE_IN_PROGRESS; // by default start up as busy
  start_sema4 = xSemaphoreCreateBinary();
  xTaskCreate(bmp085_driver_task, "bmp085_driver", BMP085_DRIVER_STACK_SIZE, NULL,
              BMP085_DRIVER_PRIORITY, NULL);
  return HACS_NO_ERROR;
}

int bmp085_request_sample(float *p_alt, int16_t *p_temp, bmp085_cb_t done_cb) {
  // Check for locking
  if (sample_lock == SAMPLE_IN_PROGRESS) {
    return -HACS_ERR_ALREADY_IN_USE;
  }

  singleshot_done_cb = done_cb;
  p_singleshot_alt = p_alt;
  p_singleshot_temp = p_temp;

  // Notify the driver task to start sampling and conversion
  // TODO: Consider using Direct Task Notification in the future
  xSemaphoreGive(start_sema4);

  return HACS_NO_ERROR;
}

static int read_hword_msb_first(uint8_t reg, uint16_t *result) {
  int retval;
  uint8_t buf[2];

  retval = i2c_master_read_mem(HACS_I2C, BMP085_ADDR, reg, buf, sizeof(buf));
  if (retval == HACS_NO_ERROR) {
    *result = (((uint16_t)buf[0]) << 8) | buf[1];
  }
  return retval;
}

static int bmp085_late_init(void) {
  int retval;

  // Read calibration data
  retval = read_hword_msb_first(0xAA, (uint16_t*)&ac1);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = read_hword_msb_first(0xAC, (uint16_t*)&ac2);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = read_hword_msb_first(0xAE, (uint16_t*)&ac3);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = read_hword_msb_first(0xB0, (uint16_t*)&ac4);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = read_hword_msb_first(0xB2, (uint16_t*)&ac5);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = read_hword_msb_first(0xB4, (uint16_t*)&ac6);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = read_hword_msb_first(0xB6, (uint16_t*)&b1);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = read_hword_msb_first(0xB8, (uint16_t*)&b2);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = read_hword_msb_first(0xBA, (uint16_t*)&mb);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = read_hword_msb_first(0xBC, (uint16_t*)&mc);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = read_hword_msb_first(0xBE, (uint16_t*)&md);

done:
  return retval;
}

static int bmp085_get_temperature(int16_t *p_temp, int32_t *b5) {
  int retval;
  uint8_t wbuf[] = {BMP085_CTRL_REG, BMP085_READ_UT};
  uint16_t ut;
  int32_t x1, x2;

  // Start the temperature conversion
  retval = i2c_master_write(HACS_I2C, BMP085_ADDR, wbuf, sizeof(wbuf));
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);

  // Wait for conversion to complete (4.5ms max)
  vTaskDelay(MS_TO_TICKS(BMP085_TEMPERATURE_CONVERSION_MS));

  // Get the raw reading
  retval = read_hword_msb_first(BMP085_MSB_REG, &ut);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);

  /* Calibrate the temperature reading
   * Source: https://www.sparkfun.com/tutorials/253
   */
  x1 = (((int32_t)ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
  x2 = ((int32_t)mc << 11) / (x1 + md);
  *b5 = x1 + x2;

  if (p_temp != NULL) *p_temp = ((*b5 + 8) >> 4);

done:
  return retval;
}

static int bmp085_get_pressure(int32_t b5, int32_t *p_pres) {
  int retval;
  uint8_t buf[3];
  uint32_t up;
  int32_t x1, x2, x3, b3, b6, p;
  uint32_t b4, b7;

  // Start the pressure conversion
  buf[0] = BMP085_CTRL_REG;
  buf[1] = BMP085_READ_UP + (BMP085_OSS << 6);
  retval = i2c_master_write(HACS_I2C, BMP085_ADDR, buf, 2);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);

  // Wait for conversion to complete (depending on the OSS setting)
  vTaskDelay(MS_TO_TICKS(2 + (3 << BMP085_OSS)));

  // Get the raw reading
  retval = i2c_master_read_mem(HACS_I2C, BMP085_ADDR, BMP085_MSB_REG,
                               buf, sizeof(buf));
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  up = ((((uint32_t)buf[0]) << 16) | (((uint32_t)buf[1]) << 8) |
        (uint32_t)buf[2]) >> (8 - BMP085_OSS);

  /* Calibrate the pressure reading.
   * Source: https://www.sparkfun.com/tutorials/253
   */
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6) >> 12) >> 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t)ac1) * 4 + x3) << BMP085_OSS) + 2) >> 2;

  // Calculate B4
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;

  b7 = ((uint32_t)(up - b3) * (50000 >> BMP085_OSS));
  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;

  // Report final result
  if (p_pres != NULL) *p_pres = p;

done:
  return retval;
}

static int bmp085_get_temp_and_press(int16_t *p_temp, int32_t *p_pres) {
  int retval;
  int32_t b5_temp;

  retval = bmp085_get_temperature(p_temp, &b5_temp);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);

  retval = bmp085_get_pressure(b5_temp, p_pres);
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);

done:
  return retval;
}

static float pressure_to_altitude(int32_t pres) {
  float altitude;
  float x;
  x = pow((float)pres / (float)press_ref , 1.0 / 5.255);
  altitude = 44330.0 * (1.0 - x);
  return altitude;
}
