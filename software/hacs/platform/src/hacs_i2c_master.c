#include "hacs_platform.h"
#include "hacs_i2c_master.h"

#define I2C_OP_TIMEOUT_MS   (HAL_MAX_DELAY)

static I2C_HandleTypeDef i2c_handles[HACS_NUM_I2C_PERIPH];

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

  return HAL_I2C_Init(hi2c);
}

int i2c_master_write(hacs_i2c_t bus, uint16_t addr, uint8_t *wbuf, size_t wsize) {
	return HAL_I2C_Master_Transmit(&i2c_handles[bus], addr, wbuf, wsize, I2C_OP_TIMEOUT_MS);
}

int i2c_master_receive(hacs_i2c_t bus, uint16_t addr, uint8_t *rbuf, size_t rsize) {
	return HAL_I2C_Master_Receive(&i2c_handles[bus], addr, rbuf, rsize, I2C_OP_TIMEOUT_MS);
}

int i2c_master_read_mem(hacs_i2c_t bus, uint16_t dev_addr, uint16_t mem_addr, uint8_t *rbuf, size_t rsize) {
	return HAL_I2C_Mem_Read(&i2c_handles[bus], dev_addr, mem_addr, I2C_MEMADD_SIZE_8BIT, rbuf, rsize, I2C_OP_TIMEOUT_MS);
}