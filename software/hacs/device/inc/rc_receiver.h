#ifndef _HACS_RC_RECEIVER_H_
#define _HACS_RC_RECEIVER_H_

#include "hacs_platform_resources.h"

int rc_recvr_init(void);
// Return number of microseconds that the pulse is high
uint32_t rc_recvr_read_chan_raw(hacs_rc_chan_t rc_chan);
// Return a reading between HACS_RC_VAL_MIN and HACS_RC_VAL_MAX
int32_t rc_recvr_read_chan_scaled(hacs_rc_chan_t rc_chan);
int rc_recvr_wait_for_sample(void);

#endif
