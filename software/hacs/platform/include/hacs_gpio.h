/*
 * Interface for a simple GPIO driver which supports only output/input mode.
 * All other modes (like alternate function and analog) should be taken care
 * of by the respective drivers. For example, the initialization of the pins
 * used by SPI should be handled by the implementation of hacs_spi_master.h. In
 * the case of STM32, hacs_spi_master makes a call to HAL_SPI_Init() which
 * invokes HAL_Msp_Init() internally, which handles the GPIO init and AF mapping.
 */

#ifndef _HACS_GPIO_H_
#define _HACS_GPIO_H_

#include "hacs_platform_resources.h"

typedef enum {
  HACS_GPIO_MODE_OUTPUT_PP = 0,
  HACS_GPIO_MODE_OUTPUT_OD,
  HACS_GPIO_MODE_INPUT,
} gpio_mode_t;

typedef enum {
  HACS_GPIO_PULL_UP = 0,
  HACS_GPIO_PULL_DOWN,
  HACS_GPIO_NO_PULL,
} gpio_pupd_t;

void gpio_init_pin(gpio_port_t port, gpio_pin_t pin, 
                   gpio_mode_t mode, gpio_pupd_t pupd);

void gpio_write_high(gpio_port_t port, gpio_pin_t pin);

void gpio_write_low(gpio_port_t port, gpio_pin_t pin);

uint8_t gpio_read_pin(gpio_port_t port, gpio_pin_t pin);

uint32_t gpio_read_port(gpio_port_t port);

#endif
