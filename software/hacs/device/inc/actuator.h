#ifndef _HACS_ACTUATOR_H_
#define _HACS_ACTUATOR_H_

#include "hacs_platform_resources.h"

int actuator_init(void);
// Set the raw servo output in terms of PWM width in microseconds
int actuator_set_raw_us(hacs_actuator_t chan, uint32_t width);
// Set the output of an actuator channel. Output value must be
// between HACS_RC_VAL_MIN and HACS_RC_VAL_MAX (For THROTTLE, val
// must be between 0 and HACS_RC_VAL_MAX). The actual output
// raw value is subject to trim values and explicit offsets.
int actuator_set_output_scaled(hacs_actuator_t chan, int32_t val);

void actuator_set_offset(hacs_actuator_t chan, int32_t offset);

void actuator_reload_trimval_from_pstore(void);

int32_t actuator_get_output_us(hacs_actuator_t chan);

#endif
