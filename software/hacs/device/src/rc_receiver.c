#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "rc_receiver.h"
#include "hacs_timer.h"
#include "hacs_gpio.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define FIRST_CHANNEL   RC_CHAN_THROTTLE
#define LAST_CHANNEL    RC_CHAN_AUX_1

#define PULSE_WIDTH_OFFSET_US ((RC_PWM_MAX_WIDTH_US+RC_PWM_MIN_WIDTH_US)/2)

//static int32_t rc_chan_offset[HACS_NUM_RC_CHAN];
static volatile uint32_t start_time[HACS_NUM_RC_CHAN];
static volatile uint32_t rc_chan_readings[HACS_NUM_RC_CHAN];
static volatile uint8_t seen_rise[HACS_NUM_RC_CHAN];
static volatile uint8_t need_sync = 1;
static xSemaphoreHandle sample_complete;

static int32_t pulse_width_to_chan_reading(hacs_rc_chan_t chan, uint32_t width_us)
{
  return (int32_t)width_us - (int32_t)PULSE_WIDTH_OFFSET_US;
}

static void decode(hacs_rc_chan_t chan)
{
  if (!seen_rise[chan]) {
    seen_rise[chan] = 1;

    // If this is the first channel, start the counting timer from 0
    if (chan == FIRST_CHANNEL) {
      timer_reset_n_go(HACS_BASIC_TIMER);

      // If this is the first frame, need to enable detection for all other channels
      if (need_sync) {
        need_sync = 0;
        gpio_exti_enable(rc_chan_to_gpio_map[RC_CHAN_THROTTLE].port,
                         rc_chan_to_gpio_map[RC_CHAN_THROTTLE].pin,
                         1, 0);
        gpio_exti_enable(rc_chan_to_gpio_map[RC_CHAN_AILERON].port,
                         rc_chan_to_gpio_map[RC_CHAN_AILERON].pin,
                         1, 0);
        gpio_exti_enable(rc_chan_to_gpio_map[RC_CHAN_ELEVATOR].port,
                         rc_chan_to_gpio_map[RC_CHAN_ELEVATOR].pin,
                         1, 0);
        gpio_exti_enable(rc_chan_to_gpio_map[RC_CHAN_RUDDER].port,
                         rc_chan_to_gpio_map[RC_CHAN_RUDDER].pin,
                         1, 0);
        gpio_exti_enable(rc_chan_to_gpio_map[RC_CHAN_AUX_0].port,
                         rc_chan_to_gpio_map[RC_CHAN_AUX_0].pin,
                         1, 0);
        gpio_exti_enable(rc_chan_to_gpio_map[RC_CHAN_AUX_1].port,
                         rc_chan_to_gpio_map[RC_CHAN_AUX_1].pin,
                         1, 0);
      }
    }

    // Remember the current time stamp and get ready for falling edge detection
    start_time[chan] = timer_get_us(HACS_BASIC_TIMER);
    gpio_exti_enable(rc_chan_to_gpio_map[chan].port,
                     rc_chan_to_gpio_map[chan].pin,
                     0, 1);
  } else {
    seen_rise[chan] = 0;

    // Get the channel reading
    rc_chan_readings[chan] = timer_get_us(HACS_BASIC_TIMER) - start_time[chan];

    // Get ready for the next detection on this channel (typically after 20ms)
    gpio_exti_enable(rc_chan_to_gpio_map[chan].port,
                     rc_chan_to_gpio_map[chan].pin,
                     1, 0);

    // If this is the last channel, stop the timer and signal sampling complete
    if (chan == LAST_CHANNEL) {
      portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

      timer_stop(HACS_BASIC_TIMER);
      xSemaphoreGiveFromISR(sample_complete, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

static void throt_chan_handler(void)
{
  decode(RC_CHAN_THROTTLE);
}

static void aeil_chan_handler(void)
{
  decode(RC_CHAN_AILERON);
}

static void elev_chan_handler(void)
{
  decode(RC_CHAN_ELEVATOR);
}

static void rudder_chan_handler(void)
{
  decode(RC_CHAN_RUDDER);
}

static void aux0_chan_handler(void)
{
  decode(RC_CHAN_AUX_0);
}

static void aux1_chan_handler(void)
{
  decode(RC_CHAN_AUX_1);
}

int rc_recvr_init(void)
{
  sample_complete = xSemaphoreCreateBinary();

  // Initialize rc channel offsets. Read them from env if valid
  // TODO

  // Set counting timer expire time to 1s
  timer_set_period(HACS_BASIC_TIMER, 1000000);

  gpio_init_pin(RC_CHAN1_PORT, RC_CHAN1_PIN, HACS_GPIO_MODE_INPUT,
                HACS_GPIO_NO_PULL);
  gpio_init_pin(RC_CHAN2_PORT, RC_CHAN2_PIN, HACS_GPIO_MODE_INPUT,
                HACS_GPIO_NO_PULL);
  gpio_init_pin(RC_CHAN3_PORT, RC_CHAN3_PIN, HACS_GPIO_MODE_INPUT,
                HACS_GPIO_NO_PULL);
  gpio_init_pin(RC_CHAN4_PORT, RC_CHAN4_PIN, HACS_GPIO_MODE_INPUT,
                HACS_GPIO_NO_PULL);
  gpio_init_pin(RC_CHAN5_PORT, RC_CHAN5_PIN, HACS_GPIO_MODE_INPUT,
                HACS_GPIO_NO_PULL);
  gpio_init_pin(RC_CHAN6_PORT, RC_CHAN6_PIN, HACS_GPIO_MODE_INPUT,
                HACS_GPIO_NO_PULL);

  gpio_exti_init(rc_chan_to_gpio_map[RC_CHAN_THROTTLE].port,
                 rc_chan_to_gpio_map[RC_CHAN_THROTTLE].pin,
                 throt_chan_handler);
  gpio_exti_init(rc_chan_to_gpio_map[RC_CHAN_AILERON].port,
                 rc_chan_to_gpio_map[RC_CHAN_AILERON].pin,
                 aeil_chan_handler);
  gpio_exti_init(rc_chan_to_gpio_map[RC_CHAN_ELEVATOR].port,
                 rc_chan_to_gpio_map[RC_CHAN_ELEVATOR].pin,
                 elev_chan_handler);
  gpio_exti_init(rc_chan_to_gpio_map[RC_CHAN_RUDDER].port,
                 rc_chan_to_gpio_map[RC_CHAN_RUDDER].pin,
                 rudder_chan_handler);
  gpio_exti_init(rc_chan_to_gpio_map[RC_CHAN_AUX_0].port,
                 rc_chan_to_gpio_map[RC_CHAN_AUX_0].pin,
                 aux0_chan_handler);
  gpio_exti_init(rc_chan_to_gpio_map[RC_CHAN_AUX_1].port,
                 rc_chan_to_gpio_map[RC_CHAN_AUX_1].pin,
                 aux1_chan_handler);

  // Use the Throttle channel (assumed to be the first channel) for synchronization
  gpio_exti_enable(rc_chan_to_gpio_map[RC_CHAN_THROTTLE].port,
                   rc_chan_to_gpio_map[RC_CHAN_THROTTLE].pin,
                   1, 0);
  
  return HACS_NO_ERROR;
}

uint32_t rc_recvr_read_chan_raw(hacs_rc_chan_t rc_chan)
{
  return rc_chan_readings[rc_chan];
}

int32_t rc_recvr_read_chan_scaled(hacs_rc_chan_t rc_chan)
{
  return pulse_width_to_chan_reading(rc_chan, rc_chan_readings[rc_chan]);
}

int rc_recvr_wait_for_sample(void)
{
  return xSemaphoreTake(sample_complete, portMAX_DELAY);
}
