#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_i2c_master.h"
#include "hmc5883.h"

int hmc5883_init() {
	static uint8_t wbuf1[] = {HMC5883_CRA_REG, 0x70}; // clear CRA7; others default
	static uint8_t wbuf2[] = {HMC5883_MR_REG, 0x00}; // continuous measurement mode
	int retval = 0;

	retval = i2c_master_write(HACS_I2C, HMC5883_ADDR, 
														wbuf1, sizeof(wbuf1));
	if (retval == 0) {
		retval = i2c_master_write(HACS_I2C, HMC5883_ADDR, 
															wbuf2, sizeof(wbuf2));
	}

	if (retval == 0) {
		retval = hmc5883_is_ready();
	}

	return retval;
}

int hmc5883_is_ready() {
	uint8_t id[] = {0, 0, 0};
	int retval = 0;

	retval = i2c_master_read_mem(HACS_I2C, HMC5883_ADDR, HMC5883_IRA_REG,
															 id, sizeof(id));

	if (id[0] == 72 && id[1] == 52 && id[2] == 51) {
		return retval;
	} else {
		printf("HMC5883: Invalid IDs %d %d %d\n",id[0],id[1],id[2]);
		return -42;
	}
}

int hmc5883_update_xyz(uint16_t *px, uint16_t *py, uint16_t *pz) {
	union {
		uint8_t buf[6];
		uint16_t vals[3];
	} mag;
	int retval = 0;

	retval = i2c_master_read_mem(HACS_I2C, HMC5883_ADDR, HMC5883_DXA_REG, 
															 mag.buf, sizeof(mag.buf));

	*px = mag.vals[0];
	*pz = mag.vals[1];
	*py = mag.vals[2];

	return retval;
}