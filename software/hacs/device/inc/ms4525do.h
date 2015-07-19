#ifndef _MS4525DO_H_
#define _MS4525DO_H_

#define MS4525DO_I2C_ADDRESS		(0x50) // left-shifted address

int ms4525do_init(void);
// Get differential pressure reading in Pa. Blocks until completion.
int ms4525do_get_dp(float *p_result);

#endif
