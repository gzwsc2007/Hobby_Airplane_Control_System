#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_i2c_master.h"
#include "hmc5883.h"
#include "hacs_pstore.h"

static float32_t hard_iron_matrix_data[HARD_IRON_MAT_ROW * HARD_IRON_MAT_COL];
static arm_matrix_instance_f32 hard_iron_matrix;

static float32_t soft_iron_matrix_data[SOFT_IRON_MAT_ROW * SOFT_IRON_MAT_COL];
static arm_matrix_instance_f32 soft_iron_matrix;

static uint8_t mag_cal_valid = 0;

static void print_mag_cal(float32_t *hard, float32_t *soft) {
  printf("hard iron: %f %f %f\r\n", hard[0], hard[1], hard[2]);
  printf("soft iron: %f %f %f; %f %f %f; %f %f %f\r\n", soft[0], soft[1], soft[2],
         soft[3], soft[4], soft[5], soft[6], soft[7], soft[8]);
}

int hmc5883_init() {
	static uint8_t wbuf1[] = {HMC5883_CRA_REG, 0x70}; // clear CRA7; others default
	static uint8_t wbuf2[] = {HMC5883_MR_REG, 0x00}; // continuous measurement mode
	int retval = 0;

	retval = i2c_master_write(HACS_I2C, HMC5883_ADDR, wbuf1, sizeof(wbuf1));
	HACS_REQUIRES(retval >= 0, done);
	retval = i2c_master_write(HACS_I2C, HMC5883_ADDR, wbuf2, sizeof(wbuf2));
	HACS_REQUIRES(retval >= 0, done);

	retval = hmc5883_is_ready();
	HACS_REQUIRES(retval >= 0, done);

	// Read calibration data from persistent storage into the xxx_matrix_data RAM buffers
  // And then call arm_mat_init()
  retval = hacs_pstore_get(HACS_PSTORE_MAG_HARD_CAL, (uint8_t*)hard_iron_matrix_data, sizeof(hard_iron_matrix_data));
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  retval = hacs_pstore_get(HACS_PSTORE_MAG_SOFT_CAL, (uint8_t*)soft_iron_matrix_data, sizeof(soft_iron_matrix_data));
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);

  arm_mat_init_f32(&hard_iron_matrix, HARD_IRON_MAT_ROW, HARD_IRON_MAT_COL, hard_iron_matrix_data);
  arm_mat_init_f32(&soft_iron_matrix, SOFT_IRON_MAT_ROW, SOFT_IRON_MAT_COL, soft_iron_matrix_data);
  mag_cal_valid = 1;
  print_mag_cal(hard_iron_matrix_data, soft_iron_matrix_data);

done:
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
		printf("HMC5883: Invalid IDs %d %d %d\n", id[0], id[1], id[2]);
		return -42;
	}
}

int hmc5883_update_xyz(int16_t *px, int16_t *py, int16_t *pz) {
	uint8_t buf[6];
	int16_t x;
	int16_t y;
	int16_t z;
	int retval = 0;

	retval = i2c_master_read_mem(HACS_I2C, HMC5883_ADDR, HMC5883_DXA_REG,
	                             buf, sizeof(buf));
	HACS_REQUIRES(retval >= 0, done);

	x = buf[0];
	x = x << 8;
	x |= buf[1];

	z = buf[2];
	z = z << 8;
	z |= buf[3];

	y = buf[4];
	y = y << 8;
	y |= buf[5];

	// Apply sign correction. TODO: this might not be a good place to do it..
	*px = y;
	*pz = z;
	*py = -x;

done:
	return retval;
}

int hmc5883_xyz_calibrated(float32_t *px, float32_t *py, float32_t *pz) {
	int16_t x,y,z;
  float32_t in_data[3];
  float32_t inter_data[3];
  float32_t out_data[3];
  arm_matrix_instance_f32 in_vect = {3, 1, in_data};
  arm_matrix_instance_f32 inter_vect = {3, 1, inter_data};
  arm_matrix_instance_f32 out_vect = {3, 1, out_data};
  int retval;

  // Obtain raw reading
  retval = hmc5883_update_xyz(&x, &y, &z);
  HACS_REQUIRES(retval >= 0, done);
  in_data[0] = (float32_t)x;
  in_data[1] = (float32_t)y;
  in_data[2] = (float32_t)z;

  if (!mag_cal_valid) {
    *px = in_data[0];
    *py = in_data[1];
    *pz = in_data[2];
    return -HACS_CAL_INVALID;
  }

  // Calibrate the raw reading
  arm_mat_add_f32(&in_vect, &hard_iron_matrix, &inter_vect);
  arm_mat_mult_f32(&soft_iron_matrix, &inter_vect, &out_vect);
  *px = out_data[0];
  *py = out_data[1];
  *pz = out_data[2];

done:
	return retval;
}

int hmc5883_set_cal(float32_t *hard_iron_data, float32_t *soft_iron_data) {
  mag_cal_valid = 0;

  // Update RAM-cached calibration data
  memcpy(hard_iron_matrix_data, hard_iron_data, sizeof(float32_t) * HARD_IRON_MAT_ROW * HARD_IRON_MAT_COL);
  arm_mat_init_f32(&hard_iron_matrix, HARD_IRON_MAT_ROW, HARD_IRON_MAT_COL, hard_iron_matrix_data);

  memcpy(soft_iron_matrix_data, soft_iron_data, sizeof(float32_t) * SOFT_IRON_MAT_ROW * SOFT_IRON_MAT_COL);
  arm_mat_init_f32(&soft_iron_matrix, SOFT_IRON_MAT_ROW, SOFT_IRON_MAT_COL, soft_iron_matrix_data);

  mag_cal_valid = 1;

  // Save new calibration data to persistent storage
  hacs_pstore_set(HACS_PSTORE_MAG_HARD_CAL, (uint8_t*)hard_iron_matrix_data, sizeof(hard_iron_matrix_data));
  hacs_pstore_set(HACS_PSTORE_MAG_SOFT_CAL, (uint8_t*)soft_iron_matrix_data, sizeof(soft_iron_matrix_data));

  return HACS_NO_ERROR;
}
