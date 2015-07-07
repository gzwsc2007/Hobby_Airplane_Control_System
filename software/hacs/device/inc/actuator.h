#ifndef _HACS_ACTUATOR_H_
#define _HACS_ACTUATOR_H_

int actuator_init(void);
// Set the output of an actuator channel. Output value must be
// between -1000 and 1000
int actuator_set_output(hacs_actuator_t chan, int32_t val);

#endif
