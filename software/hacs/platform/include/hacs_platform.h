#ifndef _HACS_PLATFORM_H_
#define _HACS_PLATFORM_H_

#include "hacs_error_codes.h"
#include "stm32f411xe.h"
#include "core_cm4.h"

#define MS_TO_TICKS(ms)                 ((ms * configTICK_RATE_HZ + 999) / 1000)

#define ALIGN_TO_WORD(addr)             (((addr) + 3) / 4 * 4) // Assume word size is 4

#define HACS_REQUIRES(__condition__, __label__) do { if (!(__condition__)) goto __label__; } while(0)

#define HACS_PSTORE_0_ADDR    ((uint32_t)0x08004000) // Sector 1
#define HACS_PSTORE_0_SECTOR	(FLASH_SECTOR_1)
#define HACS_PSTORE_1_ADDR    ((uint32_t)0x08008000) // Sector 2
#define HACS_PSTORE_1_SECTOR	(FLASH_SECTOR_2)
#define HACS_APP_BASE_ADDR    ((uint32_t)0x0800C000) // Sector 3

#define HACS_PSTORE_BANK_SIZE ((uint32_t)1024)

// Assumed convention of RC signals
#define RC_PWM_PERIOD_US     (20000UL) // 20ms
#define RC_PWM_MAX_WIDTH_US  (2000UL) // 2ms
#define RC_PWM_MIN_WIDTH_US  (1000UL) // 1ms

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
