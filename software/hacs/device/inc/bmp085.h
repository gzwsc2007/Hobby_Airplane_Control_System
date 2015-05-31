#ifndef _BMP085_H_
#define _BMP085_H_

#define BMP085_OSS          0x03 // oversampling setting (3 is high-resolution)
#define BMP085_ADDR         0xEE // shifted 7-bit address. Lowest bit always 0
#define BMP085_CTRL_REG     0xF4 // address of the control register
#define BMP085_MSB_REG      0xF6
#define BMP085_LSB_REG      0xF7
#define BMP085_XLSB_REG     0xF8
#define BMP085_READ_UT      0x2E // command for starting temperature conversion (put in reg 0xf4)
#define BMP085_READ_UP      0x34 // command for starting pressure conversion

typedef void (*bmp085_cb_t)(int retval);

int bmp085_early_init(void);

/*
 * Request one altitude and temperature reading from BMP085.
 *
 * The provided callback will be invoked upon conversion completion. The parameter
 * passed to the callback indicates whether the conversion was successful.
 */
int bmp085_request_sample(float *p_altitude, int16_t *p_temperature, bmp085_cb_t done_cb);

#endif
