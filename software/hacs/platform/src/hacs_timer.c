#include "hacs_platform.h"
#include "hacs_timer.h"
#include "stm32f4xx_hal.h"

#define TIMER_RESOLUTION_FREQ   (1000000UL) // equivalent to 1/TIMER_RESOLUTION_SECONDS

static TIM_HandleTypeDef tim_handles[HACS_NUM_TIMER_PERIPH];
static hacs_timer_cb_t tim_overflow_cb[HACS_NUM_TIMER_PERIPH];
static uint32_t pwm_chan_to_tim_chan[] = {
  [HACS_PWM_CHAN_1] = TIM_CHANNEL_1,
  [HACS_PWM_CHAN_2] = TIM_CHANNEL_2,
  [HACS_PWM_CHAN_3] = TIM_CHANNEL_3,
  [HACS_PWM_CHAN_4] = TIM_CHANNEL_4,
  [HACS_PWM_CHAN_5] = TIM_CHANNEL_1,
  [HACS_PWM_CHAN_6] = TIM_CHANNEL_2,
  [HACS_PWM_CHAN_7] = TIM_CHANNEL_3,
  [HACS_PWM_CHAN_8] = TIM_CHANNEL_4,
};

int timer_init(hacs_timer_t tim, hacs_timer_mode_t mode)
{
  uint32_t clock_freq;
  uint32_t presc;
  TIM_HandleTypeDef *htim = &tim_handles[tim];
  TIM_TypeDef *inst = hacs_tim_instances[tim];

  tim_overflow_cb[tim] = NULL;

  // First, find the appropriate prescaler to achieve microsecond resolution
  if (inst == TIM1 || inst == TIM10 || inst == TIM11) {
    // For stm32f411, these 3 timers are on the APB2 bus
    clock_freq = HAL_RCC_GetPCLK2Freq();
  } else {
    clock_freq = HAL_RCC_GetPCLK1Freq();
  }
  presc = clock_freq / TIMER_RESOLUTION_FREQ;

  // Setup timer base
  htim->Instance = inst;
  htim->Init.Prescaler = presc;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = 0;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(htim);

  if (mode == HACS_TIMER_MODE_PWM) {
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    // NOTE: Assume there are 4 PWM channels on each timer, and that all of them are used.
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_4);
  }

  return HACS_NO_ERROR;
}

int timer_set_update_cb(hacs_timer_t tim, hacs_timer_cb_t overflow_cb)
{
  TIM_TypeDef *inst = hacs_tim_instances[tim];

  // Enable timer overflow interrupt if needed
  tim_overflow_cb[tim] = overflow_cb;
  if (overflow_cb != NULL) {
    // Configure NVIC on the appropriate IRQ
    IRQn_Type irq;

    if (inst == TIM1 || inst == TIM10) irq = TIM1_UP_TIM10_IRQn;
    else if (inst == TIM2) irq = TIM2_IRQn;
    else if (inst == TIM3) irq = TIM3_IRQn;
    else if (inst == TIM4) irq = TIM4_IRQn;
    else if (inst == TIM5) irq = TIM5_IRQn;
    else if (inst == TIM9) irq = TIM1_BRK_TIM9_IRQn;
    else if (inst == TIM11) irq = TIM1_TRG_COM_TIM11_IRQn;

    NVIC_SetPriority(irq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3);
    NVIC_EnableIRQ(irq);

    __HAL_TIM_ENABLE_IT(&tim_handles[tim], TIM_IT_UPDATE);
  }
  
  return HACS_NO_ERROR;
}

int timer_set_period(hacs_timer_t tim, uint32_t us)
{
  __HAL_TIM_SetAutoreload(&tim_handles[tim], us);
  return HACS_NO_ERROR;
}

// Clear timer counter to 0, and then start the timer
int timer_reset_n_go(hacs_timer_t tim)
{
  TIM_HandleTypeDef *htim = &tim_handles[tim];

  __HAL_TIM_DISABLE(htim);
  __HAL_TIM_SetCounter(htim, 0);
  __HAL_TIM_ENABLE(htim);
  return HACS_NO_ERROR;
}

// return microseconds since timer starts
uint32_t timer_get_us(hacs_timer_t tim)
{
  // In the current implementation, each tick count is equivalent to 1 us.
  return __HAL_TIM_GetCounter(&tim_handles[tim]);
}

int timer_stop(hacs_timer_t tim)
{
  __HAL_TIM_DISABLE(&tim_handles[tim]);
  return HACS_NO_ERROR;
}

int timer_set_pwm_duty(hacs_pwm_chan_t pwm, float percent)
{
  hacs_timer_t tim = (pwm > HACS_PWM_CHAN_4) ? HACS_PWM_TIMER_1 : HACS_PWM_TIMER_0;
  TIM_HandleTypeDef *htim = &tim_handles[tim];
  uint32_t period = __HAL_TIM_GetAutoreload(htim);
  uint32_t compare = (uint32_t)((float)period*percent);

  __HAL_TIM_SetCompare(htim, pwm_chan_to_tim_chan[pwm], compare);

  return HACS_NO_ERROR;
}

int timer_start_pwm(hacs_pwm_chan_t pwm)
{
  hacs_timer_t tim = (pwm > HACS_PWM_CHAN_4) ? HACS_PWM_TIMER_1 : HACS_PWM_TIMER_0;
  TIM_HandleTypeDef *htim = &tim_handles[tim];

  return HAL_TIM_PWM_Start(htim, pwm_chan_to_tim_chan[pwm]);
}

static void tim_irq_handler(void)
{
  TIM_HandleTypeDef *htim;

  for (hacs_timer_t index = 0; index < HACS_NUM_TIMER_PERIPH; index++) {
    htim = &tim_handles[index];

    // Only care about update event. Code copied from stm32f4xx_tim.c
    if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)
    {
      if(__HAL_TIM_GET_ITSTATUS(htim, TIM_IT_UPDATE) !=RESET)
      {
        __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
        if (tim_overflow_cb[index] != NULL) tim_overflow_cb[index]();
      }
    }
  }
}

void TIM1_UP_TIM10_IRQHandler(void)
{
  tim_irq_handler();
}

void TIM2_IRQHandler(void)
{
  tim_irq_handler();
}

void TIM3_IRQHandler(void)
{
  tim_irq_handler();
}

void TIM4_IRQHandler(void)
{
  tim_irq_handler();
}

void TIM5_IRQHandler(void)
{
  tim_irq_handler();
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
  tim_irq_handler();
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  tim_irq_handler();
}
