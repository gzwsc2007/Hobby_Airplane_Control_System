#ifndef _HMC5883_H_
#define _HMC5883_H_

#define HMC5883_ADDR        0x3C // shifted 7-bit address. Lowest bit always 0
#define HMC5883_CRA_REG     0x00
#define HMC5883_CRB_REG     0x01
#define HMC5883_MR_REG      0x02
#define HMC5883_DXA_REG			0x03
#define HMC5883_DXB_REG			0x04
#define HMC5883_DZA_REG			0x05
#define HMC5883_DZB_REG			0x06
#define HMC5883_DYA_REG			0x07
#define HMC5883_DYB_REG			0x08
#define HMC5883_IRA_REG     0x0A // Identification Register A (reads "H")
#define HMC5883_IRB_REG     0x0B // reads "4"
#define HMC5883_IRC_REG     0x0C // reads "3"

#include "arm_math.h"

int hmc5883_init();
int hmc5883_is_ready();

int hmc5883_update_xyz(int16_t *px, int16_t *py, int16_t *pz);
int hmc5883_xyz_calibrated(float32_t *px, float32_t *py, float32_t *pz);
int hmc5883_set_cal(float32_t *hard_iron_data, float32_t *soft_iron_data);

#endif
