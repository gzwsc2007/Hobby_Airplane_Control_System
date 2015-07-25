#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_pstore.h"
#include "actuator.h"
#include "hacs_timer.h"

static int32_t trim_vals_us[HACS_NUM_ACTUATOR]; // trim vals are in PWM width (us)
static int32_t offsets_us[HACS_NUM_ACTUATOR]; // offsets in microseconds

int actuator_init(void)
{
  // Init trim values and offsets
  for (int i = 0; i < HACS_NUM_ACTUATOR; i++) {
    trim_vals_us[i] = RC_PWM_MIDSCALE_WIDTH_US;
    offsets_us[i] = 0;
  }
  trim_vals_us[HACS_ACTUATOR_THROTTLE] = RC_PWM_MIN_WIDTH_US;

  actuator_reload_trimval_from_pstore();

  timer_set_period(HACS_PWM_TIMER_0, RC_PWM_PERIOD_US);
  timer_set_period(HACS_PWM_TIMER_1, RC_PWM_PERIOD_US);

  // Initial value for all the channels
  actuator_set_raw_us(HACS_ACTUATOR_THROTTLE, trim_vals_us[HACS_ACTUATOR_THROTTLE]);
  actuator_set_raw_us(HACS_ACTUATOR_AILERON, trim_vals_us[HACS_ACTUATOR_AILERON]);
  actuator_set_raw_us(HACS_ACTUATOR_ELEVATOR, trim_vals_us[HACS_ACTUATOR_ELEVATOR]);
  actuator_set_raw_us(HACS_ACTUATOR_RUDDER, trim_vals_us[HACS_ACTUATOR_RUDDER]);
  actuator_set_raw_us(HACS_ACTUATOR_AUX_0, trim_vals_us[HACS_ACTUATOR_AUX_0]);
  actuator_set_raw_us(HACS_ACTUATOR_AUX_1, trim_vals_us[HACS_ACTUATOR_AUX_1]);
  actuator_set_raw_us(HACS_ACTUATOR_AUX_2, trim_vals_us[HACS_ACTUATOR_AUX_2]);
  actuator_set_raw_us(HACS_ACTUATOR_AUX_3, trim_vals_us[HACS_ACTUATOR_AUX_3]);

  // Start all the channels
  timer_start_pwm(HACS_PWM_CHAN_1);
  timer_start_pwm(HACS_PWM_CHAN_2);
  timer_start_pwm(HACS_PWM_CHAN_3);
  timer_start_pwm(HACS_PWM_CHAN_4);
  timer_start_pwm(HACS_PWM_CHAN_5);
  timer_start_pwm(HACS_PWM_CHAN_6);
  timer_start_pwm(HACS_PWM_CHAN_7);
  timer_start_pwm(HACS_PWM_CHAN_8);

  return HACS_NO_ERROR;
}

// Set the raw servo output in terms of PWM width in microseconds
int actuator_set_raw_us(hacs_actuator_t chan, uint32_t width)
{
  // Check for range
  if (width < RC_PWM_MIN_WIDTH_US) width = RC_PWM_MIN_WIDTH_US;
  else if (width > RC_PWM_MAX_WIDTH_US) width = RC_PWM_MAX_WIDTH_US;

  return timer_set_pwm_width_us(actuator_to_pwm_map[chan], width);
}

// Set the output of an actuator channel. Output value must be
// between HACS_RC_VAL_MIN and HACS_RC_VAL_MAX (For THROTTLE, val
// must be between 0 and HACS_RC_VAL_MAX). The actual output
// raw value is subject to trim values and explicit offsets.
int actuator_set_output_scaled(hacs_actuator_t chan, int32_t val)
{
  int32_t width;

  if (chan == HACS_ACTUATOR_THROTTLE) {
    // For throttle, range of val is 0 ~ HACS_RC_VAL_MAX, which is
    // half of the normal range. Thus scale val to compensate for that.
    val = val * 2;
  }

  // Obtain the raw width of the signal
  width = val + trim_vals_us[chan];

  // Apply offsets
  width += offsets_us[chan];

  // width is guaranteed to be positive
  return timer_set_pwm_width_us(actuator_to_pwm_map[chan], width);
}

void actuator_reload_trimval_from_pstore(void)
{
  int32_t vals[3];

  // Apply trim values found in PSTORE
  if (hacs_pstore_get(HACS_PSTORE_TRIM_VALS, (uint8_t*)vals, sizeof(vals)) == HACS_NO_ERROR) {
    trim_vals_us[HACS_ACTUATOR_AILERON] = vals[0];
    trim_vals_us[HACS_ACTUATOR_ELEVATOR] = vals[1];
    trim_vals_us[HACS_ACTUATOR_RUDDER] = vals[2];
  }
}

void actuator_set_offset(hacs_actuator_t chan, int32_t offset)
{
  offsets_us[chan] = offset;
}
