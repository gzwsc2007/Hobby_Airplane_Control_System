/*
 * Logarithmic frequency sweep
 *
 * Source 1: Aircraft System Identification Theory and Practice (Klein e Morelli), 
 *           Page 308 Equation (9.21)
 * Source 2: http://www.aem.umn.edu/~dorobantu/Dorobantu_et_al_2013_JA_SystemID.pdf
 *           Equation (20)
 */
#include <math.h>
#include "hacs_platform.h"
#include "hacs_system_config.h"
#include "hacs_sysid.h"
#include "rc_receiver.h"
#include "actuator.h"
#include "arm_math.h"

#define FREQ_LOWER_START_HZ     (0.1f)
#define FREQ_LOWER_END_HZ       (4.0f)
#define FREQ_HIGHER_START_HZ    (3.0f)
#define FREQ_HIGHER_END_HZ      (10.0f)

static uint32_t t0_ms; // start time
static uint32_t t1_ms; // end time
static float f0, f1_div_f0, T;

void hacs_sysid_start(uint32_t timestamp, uint32_t duration_ms)
{
  float f1;

  t0_ms = timestamp;
  t1_ms = timestamp + duration_ms;

  // Take the current RC inputs as trim values.
  rc_recvr_set_trim_vals();

  // Set all three channels to neutral, no offsets
  actuator_set_offset(HACS_ACTUATOR_AILERON, 0);
  actuator_set_offset(HACS_ACTUATOR_ELEVATOR, 0);
  actuator_set_offset(HACS_ACTUATOR_RUDDER, 0);
  actuator_set_output_scaled(HACS_ACTUATOR_AILERON, 0);
  actuator_set_output_scaled(HACS_ACTUATOR_ELEVATOR, 0);
  actuator_set_output_scaled(HACS_ACTUATOR_RUDDER, 0);

  // Initialize parameters
  if (hacs_get_sysid_freq() == HACS_SYSID_FREQ_LOWER) {
    f0 = FREQ_LOWER_START_HZ;
    f1 = FREQ_LOWER_END_HZ;
  } else {
    f0 = FREQ_HIGHER_START_HZ;
    f1 = FREQ_HIGHER_END_HZ;
  }

  f1_div_f0 = f1 / f0;
  T = (float)duration_ms / 1000.0f; // in seconds
}

// Generate an output val depending on the current time. The output will be directed to
// the appropriate channel (AILE, ELEV or RUDD) depending on sysid mode.
// Will set system mode back to MANUAL when the experiemnt duration has passed.
int hacs_sysid_generate_output(uint32_t timestamp, float amplitude, int32_t *pout)
{
  float t;
  float ft;
  int32_t out;
  hacs_sysid_mode_t sysid_mode = hacs_get_sysid_mode();

  // check for termination first
  if (timestamp >= t1_ms) {
    hacs_set_system_mode(HACS_MODE_MANUAL);
    return -1;
  }

  // Logarithmic frequency sweep
  t = ((float)timestamp - (float)t0_ms) / 1000.0f;
  ft = f0 * powf(f1_div_f0, t / T);
  out = (int32_t)(amplitude * arm_sin_f32(ft * t));

  // Direct output value to the appropriate actuator
  if (sysid_mode == HACS_SYSID_MODE_AILERON) {
    actuator_set_output_scaled(HACS_ACTUATOR_AILERON, out);
  } else if (sysid_mode == HACS_SYSID_MODE_ELEVATOR) {
    actuator_set_output_scaled(HACS_ACTUATOR_ELEVATOR, out);
  } else if (sysid_mode == HACS_SYSID_MODE_RUDDER) {
    actuator_set_output_scaled(HACS_ACTUATOR_RUDDER, out);
  }

  *pout = out;

  return HACS_NO_ERROR;
}
