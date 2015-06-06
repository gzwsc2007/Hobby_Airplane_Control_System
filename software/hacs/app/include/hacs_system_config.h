#ifndef _HACS_SYSTEM_CONFIG_H_
#define _HACS_SYSTEM_CONFIG_H_

typedef enum {
	HACS_GND_CMD_SET_MODE = 0,
	HACS_GND_CMD_GET_MODE,

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

hacs_mode_t hacs_get_system_mode(void);

void hacs_set_system_mode(hacs_mode_t mode);

#endif
