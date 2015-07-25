#ifndef _HACS_SYSID_H_
#define _HACS_SYSID_H_

#include "hacs_platform.h"

#define SYSTEM_IDENT_DURATION_US				(10UL * 1000UL) // experiment time 10s

#define SYSTEM_IDENT_AMPLITUDE					(HACS_RC_VAL_MAX * 4 / 5) // 4/5 of the MAX RC val

void hacs_sysid_start(uint32_t timestamp, uint32_t duration_ms);
// Generate an output val depending on the current time. The output will be directed to
// the appropriate channel (AILE, ELEV or RUDD) depending on sysid mode.
// Will set system mode back to MANUAL when the experiemnt duration has passed.
int hacs_sysid_generate_output(uint32_t timestamp, float amplitude, int32_t *pout);

#endif
