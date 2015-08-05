#ifndef _HACS_SYSTEM_CONFIG_H_
#define _HACS_SYSTEM_CONFIG_H_

typedef enum {
  HACS_GND_CMD_SET_MODE = 0,
  HACS_GND_CMD_GET_MODE,

  HACS_GND_CMD_CALIBRATE_AIRSPEED,
  HACS_GND_CMD_CALIBRATE_BAROMETER,
  HACS_GND_CMD_CALIBRATE_TRIM_VALUES,

  HACS_GND_CMD_SET_SYSID_MODE,
  HACS_GND_CMD_SET_SYSID_FREQ,

  HACS_GND_CMD_TELEM_TEST,

  HACS_GND_CMD_SENTINEL,
} hacs_gnd_cmd_t;

typedef enum {
  HACS_MODE_MANUAL = 0,
  HACS_MODE_MANUAL_WITH_SAS,
  HACS_MODE_AUTOPILOT,
  HACS_MODE_SYSTEM_IDENTIFICATION,
  HACS_MODE_MAG_CAL,

  HACS_MODE_SENTINEL,
} hacs_mode_t;

typedef enum {
  HACS_SYSID_MODE_MANUAL = 0,
  HACS_SYSID_MODE_AILERON,
  HACS_SYSID_MODE_ELEVATOR,
  HACS_SYSID_MODE_RUDDER,

  HACS_SYSID_MODE_SENTINEL,
} hacs_sysid_mode_t;

typedef enum {
  HACS_SYSID_FREQ_LOWER = 0,
  HACS_SYSID_FREQ_HIGHER,

  HACS_SYSID_FREQ_SENTINEL,
} hacs_sysid_freq_t;

hacs_mode_t hacs_get_system_mode(void);
void hacs_set_system_mode(hacs_mode_t mode);

hacs_sysid_mode_t hacs_get_sysid_mode(void);
void hacs_set_sysid_mode(hacs_sysid_mode_t sysid_mode);

hacs_sysid_freq_t hacs_get_sysid_freq(void);
void hacs_set_sysid_freq(hacs_sysid_freq_t sysid_freq);

#endif
