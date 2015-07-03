#include "hacs_platform.h"
#include "hacs_sensor_cal.h"

static float32_t hard_iron_matrix_data[HARD_IRON_MAT_ROW * HARD_IRON_MAT_COL];
static arm_matrix_instance_f32 hard_iron_matrix;

static float32_t soft_iron_matrix_data[SOFT_IRON_MAT_ROW * SOFT_IRON_MAT_COL];
static arm_matrix_instance_f32 soft_iron_matrix;

static uint8_t mag_cal_valid = 0;

int hacs_cal_init(void) {
  // Read calibration data from persistent storage into the xxx_matrix_data RAM buffers
  // And then call arm_mat_init()
  // TODO
  return HACS_NO_ERROR;
}

int hacs_cal_mag_apply(float32_t inx, float32_t iny, float32_t inz,
                       float32_t *outx, float32_t *outy, float32_t *outz) {
  float32_t in_data[3] = {inx, iny, inz};
  float32_t inter_data[3];
  float32_t out_data[3];
  arm_matrix_instance_f32 in_vect = {3, 1, in_data};
  arm_matrix_instance_f32 inter_vect = {3, 1, inter_data};
  arm_matrix_instance_f32 out_vect = {3, 1, out_data};

  if (!mag_cal_valid) {
    *outx = inx;
    *outy = iny;
    *outz = inz;
    return -HACS_CAL_INVALID;
  }

  arm_mat_add_f32(&in_vect, &hard_iron_matrix, &inter_vect);
  arm_mat_mult_f32(&soft_iron_matrix, &inter_vect, &out_vect);
  *outx = out_data[0];
  *outy = out_data[1];
  *outz = out_data[2];

  return HACS_NO_ERROR;
}

int hacs_cal_mag_config(float32_t *hard_iron_data, float32_t *soft_iron_data) {
  mag_cal_valid = 0;

  // Update RAM-cached calibration data
  memcpy(hard_iron_matrix_data, hard_iron_data, sizeof(float32_t) * HARD_IRON_MAT_ROW * HARD_IRON_MAT_COL);
  arm_mat_init_f32(&hard_iron_matrix, HARD_IRON_MAT_ROW, HARD_IRON_MAT_COL, hard_iron_matrix_data);

  memcpy(soft_iron_matrix_data, soft_iron_data, sizeof(float32_t) * SOFT_IRON_MAT_ROW * SOFT_IRON_MAT_COL);
  arm_mat_init_f32(&soft_iron_matrix, SOFT_IRON_MAT_ROW, SOFT_IRON_MAT_COL, soft_iron_matrix_data);

  mag_cal_valid = 1;

  // Save new calibration data to persistent storage
  // TODO

  return HACS_NO_ERROR;
}
