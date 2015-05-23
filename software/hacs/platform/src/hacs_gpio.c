#include "hacs_gpio.h"

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