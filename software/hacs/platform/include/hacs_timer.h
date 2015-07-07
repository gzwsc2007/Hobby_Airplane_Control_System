/*
 * A simple timer driver interface. Only support counting (with microseconds
 * resolution) and PWM output modes.
 */
#ifndef _HACS_TIMER_H_
#define _HACS_TIMER_H_

#include "hacs_platform_resources.h"

typedef void (*hacs_timer_cb_t)(void);

typedef enum {
  HACS_TIMER_MODE_COUNTING,
  HACS_TIMER_MODE_PWM,
} hacs_timer_mode_t;

int timer_init(hacs_timer_t tim, hacs_timer_mode_t mode);
int timer_set_update_cb(hacs_timer_t tim, hacs_timer_cb_t overflow_cb);

int timer_set_period(hacs_timer_t tim, uint32_t us);
int timer_reset_n_go(hacs_timer_t tim);
uint32_t timer_get_us(hacs_timer_t tim); // return microseconds since timer starts
int timer_stop(hacs_timer_t tim);

int timer_set_pwm_duty(hacs_pwm_chan_t pwm, float percent);
int timer_set_pwm_width_us(hacs_pwm_chan_t pwm, uint32_t us);
int timer_start_pwm(hacs_pwm_chan_t pwm);

#endif
