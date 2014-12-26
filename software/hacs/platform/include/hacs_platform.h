#ifndef _HACS_PLATFORM_H_
#define _HACS_PLATFORM_H_

#include "FreeRTOSConfig.h"

#define MS_TO_TICKS(ms)                 ((ms * configTICK_RATE_HZ + 999) / 1000)

#ifndef __GNUC__
static inline void __disable_irq(void) { asm("cpsid"); }
static inline void __enable_irq(void) { asm("cpsie"); }
#endif

extern uint8_t hacs_critical_ref_count;

void hacs_platform_init(void);

static inline void hacs_enter_critical(void)
{
	__disable_irq();
	hacs_critical_ref_count++;
}

static inline void hacs_exit_critical(void)
{
	hacs_critical_ref_count--;
	if (hacs_critical_ref_count == 0) {
		__enable_irq();
	}
}

#endif
