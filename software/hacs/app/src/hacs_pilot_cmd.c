#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_system_config.h"
#include "rc_receiver.h"
#include "actuator.h"

void hacs_pilot_cmd_task(void *param)
{
  hacs_mode_t mode;

  while(1) {
    rc_recvr_wait_for_sample();

    mode = hacs_get_system_mode();

    if (mode == HACS_MODE_MANUAL) {
      actuator_set_output(HACS_ACTUATOR_THROTTLE, rc_recvr_read_chan_scaled(RC_CHAN_THROTTLE));
      actuator_set_output(HACS_ACTUATOR_AILERON, rc_recvr_read_chan_scaled(RC_CHAN_AILERON));
      actuator_set_output(HACS_ACTUATOR_ELEVATOR, rc_recvr_read_chan_scaled(RC_CHAN_ELEVATOR));
      actuator_set_output(HACS_ACTUATOR_RUDDER, rc_recvr_read_chan_scaled(RC_CHAN_RUDDER));
      actuator_set_output(HACS_ACTUATOR_AUX_0, rc_recvr_read_chan_scaled(RC_CHAN_AUX_0));
      actuator_set_output(HACS_ACTUATOR_AUX_1, rc_recvr_read_chan_scaled(RC_CHAN_AUX_1));
    }
  }
}
