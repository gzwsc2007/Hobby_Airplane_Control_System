#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_system_config.h"
#include "rc_receiver.h"
#include "actuator.h"
#include "hacs_gpio.h"

void hacs_pilot_cmd_task(void *param)
{
  hacs_mode_t mode;
  uint8_t high = 0;

  // Init ST_to_tiny_checkin pin
  gpio_init_pin(ST_TO_TINY_PORT, ST_TO_TINY_PIN, HACS_GPIO_MODE_OUTPUT_PP, HACS_GPIO_NO_PULL);
  gpio_write_low(ST_TO_TINY_PORT, ST_TO_TINY_PIN);

  while(1) {
    rc_recvr_wait_for_sample();

    // Check in to watchdog tiny
    if (high) {
      gpio_write_low(ST_TO_TINY_PORT, ST_TO_TINY_PIN);
    } else {
      gpio_write_high(ST_TO_TINY_PORT, ST_TO_TINY_PIN);
    }
    high = !high;

    mode = hacs_get_system_mode();

    if (mode == HACS_MODE_MANUAL) {
      actuator_set_raw_us(HACS_ACTUATOR_THROTTLE, rc_recvr_read_chan_raw(RC_CHAN_THROTTLE));
      actuator_set_raw_us(HACS_ACTUATOR_AILERON, rc_recvr_read_chan_raw(RC_CHAN_AILERON));
      actuator_set_raw_us(HACS_ACTUATOR_ELEVATOR, rc_recvr_read_chan_raw(RC_CHAN_ELEVATOR));
      actuator_set_raw_us(HACS_ACTUATOR_RUDDER, rc_recvr_read_chan_raw(RC_CHAN_RUDDER));
      actuator_set_raw_us(HACS_ACTUATOR_AUX_0, rc_recvr_read_chan_raw(RC_CHAN_AUX_0));
      actuator_set_raw_us(HACS_ACTUATOR_AUX_1, rc_recvr_read_chan_raw(RC_CHAN_AUX_1));
    } else if (mode == HACS_MODE_SYSTEM_IDENTIFICATION) {
      hacs_sysid_mode_t sysid_mode = hacs_get_sysid_mode();

      // In system ident manual mode, allow manual control of all three channels.
      // Otherwise, only the active channel can be compensated manually.
      if (sysid_mode == HACS_SYSID_MODE_MANUAL) {
        // simple passthrough. Note that throttle is fixed.
        actuator_set_raw_us(HACS_ACTUATOR_AILERON, rc_recvr_read_chan_raw(RC_CHAN_AILERON));
        actuator_set_raw_us(HACS_ACTUATOR_ELEVATOR, rc_recvr_read_chan_raw(RC_CHAN_ELEVATOR));
        actuator_set_raw_us(HACS_ACTUATOR_RUDDER, rc_recvr_read_chan_raw(RC_CHAN_RUDDER));
      } else if (sysid_mode == HACS_SYSID_MODE_AILERON) {
        actuator_set_offset(HACS_ACTUATOR_AILERON, 
                            rc_recvr_read_chan_raw(RC_CHAN_AILERON) - rc_recvr_get_trim_val(RC_CHAN_AILERON));
      } else if (sysid_mode == HACS_SYSID_MODE_ELEVATOR) {
        actuator_set_offset(HACS_ACTUATOR_ELEVATOR, 
                            rc_recvr_read_chan_raw(RC_CHAN_ELEVATOR) - rc_recvr_get_trim_val(RC_CHAN_ELEVATOR));
      } else if (sysid_mode == HACS_SYSID_MODE_RUDDER) {
        actuator_set_offset(HACS_ACTUATOR_RUDDER, 
                            rc_recvr_read_chan_raw(RC_CHAN_RUDDER) - rc_recvr_get_trim_val(RC_CHAN_RUDDER));
      }
    }
  }
}
