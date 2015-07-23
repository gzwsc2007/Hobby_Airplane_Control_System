#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "actuator.h"
#include "hacs_timer.h"

int actuator_init(void)
{
  timer_set_period(HACS_PWM_TIMER_0, RC_PWM_PERIOD_US);
  timer_set_period(HACS_PWM_TIMER_1, RC_PWM_PERIOD_US);

  // Initial value for all the channels
  timer_set_pwm_duty(actuator_to_pwm_map[HACS_ACTUATOR_THROTTLE], RC_PWM_MIN_WIDTH_US);
  timer_set_pwm_duty(actuator_to_pwm_map[HACS_ACTUATOR_AILERON], RC_PWM_MIDSCALE_WIDTH_US);
  timer_set_pwm_duty(actuator_to_pwm_map[HACS_ACTUATOR_ELEVATOR], RC_PWM_MIDSCALE_WIDTH_US);
  timer_set_pwm_duty(actuator_to_pwm_map[HACS_ACTUATOR_RUDDER], RC_PWM_MIDSCALE_WIDTH_US);
  timer_set_pwm_duty(actuator_to_pwm_map[HACS_ACTUATOR_AUX_0], RC_PWM_MIDSCALE_WIDTH_US);
  timer_set_pwm_duty(actuator_to_pwm_map[HACS_ACTUATOR_AUX_1], RC_PWM_MIDSCALE_WIDTH_US);
  timer_set_pwm_duty(actuator_to_pwm_map[HACS_ACTUATOR_AUX_2], RC_PWM_MIDSCALE_WIDTH_US);
  timer_set_pwm_duty(actuator_to_pwm_map[HACS_ACTUATOR_AUX_3], RC_PWM_MIDSCALE_WIDTH_US);

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

// Set the output of an actuator channel. Output value must be
// between HACS_RC_VAL_MIN and HACS_RC_VAL_MAX
int actuator_set_output(hacs_actuator_t chan, int32_t val)
{
  int32_t width;

  width = val + RC_PWM_MIDSCALE_WIDTH_US;
  if (width < (int32_t)RC_PWM_MIN_WIDTH_US) width = (int32_t)RC_PWM_MIN_WIDTH_US;
  else if (width > (int32_t)RC_PWM_MAX_WIDTH_US) width = (int32_t)RC_PWM_MAX_WIDTH_US;

  // width is guaranteed to be positive
  return timer_set_pwm_width_us(actuator_to_pwm_map[chan], width);
}
