#include "hacs_system_config.h"

static hacs_mode_t system_mode = HACS_MODE_MANUAL;
static hacs_sysid_mode_t sysid_mode = HACS_SYSID_MODE_MANUAL;
static hacs_sysid_freq_t sysid_freq = HACS_SYSID_FREQ_LOWER;

hacs_mode_t hacs_get_system_mode(void) {
  return system_mode;
}

void hacs_set_system_mode(hacs_mode_t mode) {
  if (mode < HACS_MODE_SENTINEL) {
    system_mode = mode;
  }
}

hacs_sysid_mode_t hacs_get_sysid_mode(void) {
  return sysid_mode;
}

void hacs_set_sysid_mode(hacs_sysid_mode_t mode) {
  if (sysid_mode < HACS_SYSID_MODE_SENTINEL) {
    sysid_mode = mode;
  }
}

hacs_sysid_freq_t hacs_get_sysid_freq(void) {
  return sysid_freq;
}

void hacs_set_sysid_freq(hacs_sysid_freq_t freq) {
  if (sysid_freq < HACS_SYSID_FREQ_SENTINEL) {
    sysid_freq = freq;
  }
}