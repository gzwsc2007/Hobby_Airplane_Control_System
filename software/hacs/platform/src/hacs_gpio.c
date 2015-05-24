#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_gpio.h"
#include "hacs_error_codes.h"

void gpio_init_pin(gpio_port_t port, gpio_pin_t pin, 
                   gpio_mode_t mode, gpio_pupd_t pupd) {
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = pin;
  
  if (mode == HACS_GPIO_MODE_OUTPUT_PP) {
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  } else if (mode == HACS_GPIO_MODE_OUTPUT_OD) {
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  } else if (mode == HACS_GPIO_MODE_INPUT) {
  	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  }

  if (pupd == HACS_GPIO_PULL_UP) {
  	GPIO_InitStruct.Pull = GPIO_PULLUP;
  } else if (pupd == HACS_GPIO_PULL_DOWN) {
  	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  } else {
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  }
  
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = 0;

  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void gpio_write_high(gpio_port_t port, gpio_pin_t pin) {
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void gpio_write_low(gpio_port_t port, gpio_pin_t pin) {
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

uint8_t gpio_read_pin(gpio_port_t port, gpio_pin_t pin) {
	return HAL_GPIO_ReadPin(port, pin);
}

uint32_t gpio_read_port(gpio_port_t port) {
	return port->IDR;
}

/*** GPIO External Interrupt ***/

#define GPIO_NUMBER 16
#define NOT_ASSIGNED 0

static gpio_port_t exti_port_assignment[GPIO_NUMBER] = {
  NOT_ASSIGNED, NOT_ASSIGNED, NOT_ASSIGNED, NOT_ASSIGNED,
  NOT_ASSIGNED, NOT_ASSIGNED, NOT_ASSIGNED, NOT_ASSIGNED,
  NOT_ASSIGNED, NOT_ASSIGNED, NOT_ASSIGNED, NOT_ASSIGNED,
  NOT_ASSIGNED, NOT_ASSIGNED, NOT_ASSIGNED, NOT_ASSIGNED,
};

static hacs_exti_cb_t exti_cb_assignment[GPIO_NUMBER] = {
  NULL, NULL, NULL, NULL,
  NULL, NULL, NULL, NULL,
  NULL, NULL, NULL, NULL,
  NULL, NULL, NULL, NULL,
};

static uint32_t gpio_pin_to_num(gpio_pin_t pin) {
  uint8_t num = 0;
  while(pin > GPIO_PIN_0) {
    pin = pin >> 1;
    num++;
  }
  return num;
}

static uint32_t gpio_port_to_num(gpio_port_t port) {
  if (port == GPIOA) return 0;
  else if (port == GPIOB) return 1;
  else if (port == GPIOC) return 2;
  else if (port == GPIOD) return 3;
  else if (port == GPIOE) return 4;
  else if (port == GPIOH) return 7;
  else {
    printf("hacs_gpio.c: Invalid GPIO port!\r\n");
    return 0;
  }
}

int gpio_exti_init(gpio_port_t port, gpio_pin_t pin, 
                   hacs_exti_cb_t cb) {
  uint32_t pin_num = gpio_pin_to_num(pin);
  uint32_t port_num = gpio_port_to_num(port);
  uint32_t temp;
  int retval;

  // First check if the EXTI line has already been assigned.
  if (exti_port_assignment[pin_num] != NOT_ASSIGNED) {
    // Right now we just go ahead and overwrite the mapping, but
    // will return a warning code that indicates this situation..
    retval = HACS_ERROR_EXTI_OVERWRITE;
  }

  /* Enable SYSCFG Clock */
  __SYSCFG_CLK_ENABLE();

  // Set EXTI line mapping (refernece: STM32F4xx_HAL)
  temp = SYSCFG->EXTICR[pin_num >> 2];
  temp &= ~(((uint32_t)0x0F) << (4 * (pin_num & 0x03))); // clear
  temp |= (port_num << (4 * (pin_num & 0x03))); // set
  SYSCFG->EXTICR[pin_num >> 2] = temp;

  exti_port_assignment[pin_num] = port;
  exti_cb_assignment[pin_num] = cb;

  // Enable the EXTI line
  EXTI->IMR |= pin;

  return retval;
}

void gpio_exti_enable(gpio_port_t port, gpio_pin_t pin, 
                     uint8_t rise, uint8_t fall) {
  uint32_t temp;
  uint32_t pin32 = pin;
  uint32_t pin_num = gpio_pin_to_num(pin);

  // set trigger polarity
  temp = EXTI->RTSR;
  temp &= ~pin32;
  if (rise) {
    temp |= pin32;
  }
  EXTI->RTSR = temp;

  temp = EXTI->FTSR;
  temp &= ~pin32;
  if (fall) {
    temp |= pin32;
  }
  EXTI->FTSR = temp;

  // Configure NVIC on the appropriate IRQ
  IRQn_Type irq;
  if (pin_num == 0) {
    irq = EXTI0_IRQn;
  } else if (pin_num == 1) {
    irq = EXTI1_IRQn;
  } else if (pin_num == 2) {
    irq = EXTI2_IRQn;
  } else if (pin_num == 3) {
    irq = EXTI3_IRQn;
  } else if (pin_num == 4) {
    irq = EXTI4_IRQn;
  } else if (pin_num >= 5 && pin_num <= 9) {
    irq = EXTI9_5_IRQn;
  } else if (pin_num >= 10 && pin_num <= 15) {
    irq = EXTI15_10_IRQn;
  }
  NVIC_SetPriority(irq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  NVIC_EnableIRQ(irq);
}

void gpio_exti_disable(gpio_port_t port, gpio_pin_t pin) {
  uint32_t pin_num = gpio_pin_to_num(pin);

  // Configure NVIC on the appropriate IRQ
  IRQn_Type irq;
  if (pin_num == 0) {
    irq = EXTI0_IRQn;
  } else if (pin_num == 1) {
    irq = EXTI1_IRQn;
  } else if (pin_num == 2) {
    irq = EXTI2_IRQn;
  } else if (pin_num == 3) {
    irq = EXTI3_IRQn;
  } else if (pin_num == 4) {
    irq = EXTI4_IRQn;
  } else if (pin_num >= 5 && pin_num <= 9) {
    irq = EXTI9_5_IRQn;
  } else if (pin_num >= 10 && pin_num <= 15) {
    irq = EXTI15_10_IRQn;
  }

  NVIC_DisableIRQ(irq);
}

/*** EXTI IRQ Handlers ***/

void EXTI0_IRQHandler(void) {
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
  if (exti_cb_assignment[0] != NULL) {
    exti_cb_assignment[0]();
  }
}

void EXTI1_IRQHandler(void) {
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
  if (exti_cb_assignment[1] != NULL) {
    exti_cb_assignment[1]();
  }
}

void EXTI2_IRQHandler(void) {
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
  if (exti_cb_assignment[2] != NULL) {
    exti_cb_assignment[2]();
  }
}

void EXTI3_IRQHandler(void) {
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
  if (exti_cb_assignment[3] != NULL) {
    exti_cb_assignment[3]();
  }
}

void EXTI4_IRQHandler(void) {
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
  if (exti_cb_assignment[4] != NULL) {
    exti_cb_assignment[4]();
  }
}

void EXTI9_5_IRQHandler(void) {
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
    if (exti_cb_assignment[5] != NULL) {
      exti_cb_assignment[5]();
    }
  } else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
    if (exti_cb_assignment[6] != NULL) {
      exti_cb_assignment[6]();
    }
  } else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
    if (exti_cb_assignment[7] != NULL) {
      exti_cb_assignment[7]();
    }
  } else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
    if (exti_cb_assignment[8] != NULL) {
      exti_cb_assignment[8]();
    }
  } else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
    if (exti_cb_assignment[9] != NULL) {
      exti_cb_assignment[9]();
    }
  }
}

void EXTI15_10_IRQHandler(void) {
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
    if (exti_cb_assignment[10] != NULL) {
      exti_cb_assignment[10]();
    }
  } else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
    if (exti_cb_assignment[11] != NULL) {
      exti_cb_assignment[11]();
    }
  } else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
    if (exti_cb_assignment[12] != NULL) {
      exti_cb_assignment[12]();
    }
  } else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
    if (exti_cb_assignment[13] != NULL) {
      exti_cb_assignment[13]();
    }
  } else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
    if (exti_cb_assignment[14] != NULL) {
      exti_cb_assignment[14]();
    }
  } else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
    if (exti_cb_assignment[15] != NULL) {
      exti_cb_assignment[15]();
    }
  }
}
