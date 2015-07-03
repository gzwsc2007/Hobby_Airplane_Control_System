#ifndef _HACS_PLATFORM_H_
#define _HACS_PLATFORM_H_

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hacs_error_codes.h"
#include "stm32f411xe.h"
#include "core_cm4.h"

#define MS_TO_TICKS(ms)                 ((ms * configTICK_RATE_HZ + 999) / 1000)

#define HACS_REQUIRES(__condition__, __label__) do { if (!(__condition__)) goto __label__; } while(0)

extern uint8_t hacs_critical_ref_count;

void hacs_platform_init(void);

__STATIC_INLINE void hacs_enter_critical(void)
{
	__disable_irq();
	hacs_critical_ref_count++;
}

__STATIC_INLINE void hacs_exit_critical(void)
{
	hacs_critical_ref_count--;
	if (hacs_critical_ref_count == 0) {
		__enable_irq();
	}
}

// Only works @ 100MHz CPU clock
__STATIC_INLINE void delay_us(uint32_t us)
{
	for (uint32_t i = 0; i < 100 * us; i++) {
	}
}

#endif
