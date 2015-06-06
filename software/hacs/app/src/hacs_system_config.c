#include "hacs_system_config.h"

static hacs_mode_t system_mode = HACS_MODE_MANUAL;

hacs_mode_t hacs_get_system_mode(void) {
	return system_mode;
}

void hacs_set_system_mode(hacs_mode_t mode) {
	if (mode < HACS_MODE_SENTINEL) {
		system_mode = mode;
	}
}