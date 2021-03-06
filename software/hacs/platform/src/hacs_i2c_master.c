#include "hacs_platform.h"
#include "hacs_i2c_master.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define I2C_OP_TIMEOUT_MS   (100)

static I2C_HandleTypeDef i2c_handles[HACS_NUM_I2C_PERIPH];
static xSemaphoreHandle i2c_locks[HACS_NUM_I2C_PERIPH];

int i2c_master_init(hacs_i2c_t bus, uint32_t freq) {
  I2C_HandleTypeDef* hi2c = &i2c_handles[bus];

  hi2c->Instance = hacs_i2c_instances[bus];
  hi2c->Init.ClockSpeed = freq;
  hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c->Init.OwnAddress1 = 0;
  hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c->Init.OwnAddress2 = 0;
  hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

  HAL_I2C_Init(hi2c);

  // This can prevent the BUSY flag being set
  __I2C1_FORCE_RESET();
  delay_us(1);
  __I2C1_RELEASE_RESET();

  // Create a mutex for each bus
  i2c_locks[bus] = xSemaphoreCreateMutex();

  return HAL_I2C_Init(hi2c);
}

int i2c_master_write(hacs_i2c_t bus, uint16_t addr, uint8_t *wbuf, size_t wsize) {
  int retval;

  xSemaphoreTake(i2c_locks[bus], portMAX_DELAY);
  retval = HAL_I2C_Master_Transmit(&i2c_handles[bus], addr, wbuf, wsize, I2C_OP_TIMEOUT_MS);
  xSemaphoreGive(i2c_locks[bus]);

  return retval;
}

int i2c_master_receive(hacs_i2c_t bus, uint16_t addr, uint8_t *rbuf, size_t rsize) {
  int retval;

  xSemaphoreTake(i2c_locks[bus], portMAX_DELAY);
  retval = HAL_I2C_Master_Receive(&i2c_handles[bus], addr, rbuf, rsize, I2C_OP_TIMEOUT_MS);
  xSemaphoreGive(i2c_locks[bus]);

  return retval;
}

int i2c_master_read_mem(hacs_i2c_t bus, uint16_t dev_addr, uint16_t mem_addr, uint8_t *rbuf, size_t rsize) {
  int retval;

  xSemaphoreTake(i2c_locks[bus], portMAX_DELAY);
  retval = HAL_I2C_Mem_Read(&i2c_handles[bus], dev_addr, mem_addr,
                            I2C_MEMADD_SIZE_8BIT, rbuf, rsize, I2C_OP_TIMEOUT_MS);
  xSemaphoreGive(i2c_locks[bus]);

  return retval;
}