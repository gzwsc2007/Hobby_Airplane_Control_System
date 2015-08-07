#ifndef _HACS_RC_RECEIVER_H_
#define _HACS_RC_RECEIVER_H_

#include "hacs_platform_resources.h"

int rc_recvr_init(void);
// Return number of microseconds that the pulse is high
uint32_t rc_recvr_read_chan_raw(hacs_rc_chan_t rc_chan);
// Return a reading between HACS_RC_VAL_MIN and HACS_RC_VAL_MAX
int32_t rc_recvr_read_chan_scaled(hacs_rc_chan_t rc_chan);
int rc_recvr_wait_for_sample(void);
// Use the current AILE, ELEV and RUDD readings as trim values
int rc_recvr_set_trim_vals(void);
// Get trim values. rc_chan can only be AILE, ELEV or RUDD
int32_t rc_recvr_get_trim_val(hacs_rc_chan_t rc_chan);
// Set trim val cache in RAM. Does not store to PSTORE. Saving things to
// PSTORE will mess up with the RC channel readings, which is not ideal in
// some cases.
void rc_recvr_set_trim_vals_cache(void);
#endif
