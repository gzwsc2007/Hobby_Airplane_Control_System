#ifndef _HACS_SENSOR_CAL_H_
#define _HACS_SENSOR_CAL_H_

#include "arm_math.h"

#define HARD_IRON_MAT_ROW   (3)
#define HARD_IRON_MAT_COL   (1)
#define SOFT_IRON_MAT_ROW   (3)
#define SOFT_IRON_MAT_COL   (3)

int hacs_cal_init(void);

int hacs_cal_mag_apply(float32_t inx, float32_t iny, float32_t inz,
                       float32_t *outx, float32_t *outy, float32_t *outz);
int hacs_cal_mag_config(float32_t x_offset, float32_t y_offset, float32_t z_offsest,
                        float32_t *soft_iron_data);

#endif
