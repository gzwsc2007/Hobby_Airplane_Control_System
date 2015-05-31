#ifndef _HACS_I2C_MASTER_H_
#define _HACS_I2C_MASTER_H_

#include <stdlib.h>
#include "hacs_platform_resources.h"

/* Currently only supports 8-bit peripherals */

int i2c_master_init(hacs_i2c_t bus, uint32_t freq);

/* These functions should only be called from a task */
int i2c_master_write(hacs_i2c_t bus, uint16_t addr, uint8_t *wbuf, size_t wsize);
int i2c_master_receive(hacs_i2c_t bus, uint16_t addr, uint8_t *rbuf, size_t rsize);
int i2c_master_read_mem(hacs_i2c_t bus, uint16_t dev_addr, uint16_t mem_addr, uint8_t *rbuf, size_t rsize);

#endif
