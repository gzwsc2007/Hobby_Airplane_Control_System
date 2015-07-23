#ifndef _HACS_PIN_DEFINES_H_
#define _HACS_PIN_DEFINES_H_

#include "stm32f4xx_hal.h"

typedef GPIO_TypeDef* gpio_port_t;
typedef uint16_t gpio_pin_t;
typedef struct {
  gpio_port_t port;
  gpio_pin_t  pin;
} gpio_port_pin_t;

/*** SPI ***/
#define HACS_NUM_SPI_PERIPH       (2)
extern SPI_TypeDef* const hacs_spi_instances[HACS_NUM_SPI_PERIPH];
extern GPIO_TypeDef* const hacs_spi_cs_port[HACS_NUM_SPI_PERIPH];
extern const uint16_t hacs_spi_cs_pin[HACS_NUM_SPI_PERIPH];

// hacs_spi_t is defined as an index
typedef enum {
  HACS_SPI_NRF24 = 0,
  HACS_SPI_ADS1120,
} hacs_spi_t;

/*** I2C ***/
#define HACS_NUM_I2C_PERIPH       (1)
extern I2C_TypeDef* const hacs_i2c_instances[HACS_NUM_I2C_PERIPH];

// hacs_i2c_t is defined as an index
typedef enum {
  HACS_I2C = 0,
} hacs_i2c_t;

/*** UART ***/
#define HACS_NUM_UART_PERIPH      (2)
extern USART_TypeDef* const hacs_uart_instances[HACS_NUM_UART_PERIPH];
extern DMA_Stream_TypeDef* const hacs_uart_rx_dma_stream[HACS_NUM_UART_PERIPH];
extern const uint32_t hacs_uart_rx_dma_chan[HACS_NUM_UART_PERIPH];
extern const IRQn_Type hacs_uart_rx_dma_irq[HACS_NUM_UART_PERIPH];

// hacs_uart_t is defined as an index
typedef enum {
  HACS_UART_MPU6050 = 0,
  HACS_UART_GPS,
} hacs_uart_t;

/*** TIMER ***/
#define HACS_NUM_TIMER_PERIPH			(3)
extern TIM_TypeDef* const hacs_tim_instances[HACS_NUM_TIMER_PERIPH];

typedef enum {
  HACS_BASIC_TIMER,
  HACS_PWM_TIMER_0,
  HACS_PWM_TIMER_1,
} hacs_timer_t;

// The below 8 channels should correspond to the channel numbers imprinted
// on the HACS shield.
typedef enum {
  HACS_PWM_CHAN_1 = 0,
  HACS_PWM_CHAN_2,
  HACS_PWM_CHAN_3,
  HACS_PWM_CHAN_4,
  HACS_PWM_CHAN_5,
  HACS_PWM_CHAN_6,
  HACS_PWM_CHAN_7,
  HACS_PWM_CHAN_8,

  HACS_NUM_PWM_CHAN,
} hacs_pwm_chan_t;

/*** RC Receiver ***/
typedef enum {
  RC_CHAN_THROTTLE = 0,
  RC_CHAN_AILERON,
  RC_CHAN_ELEVATOR,
  RC_CHAN_RUDDER,
  RC_CHAN_AUX_0,
  RC_CHAN_AUX_1,

  HACS_NUM_RC_CHAN,
} hacs_rc_chan_t;
extern const gpio_port_pin_t rc_chan_to_gpio_map[HACS_NUM_RC_CHAN];

/*** Actuator ***/
typedef enum {
  HACS_ACTUATOR_THROTTLE = 0,
  HACS_ACTUATOR_AILERON,
  HACS_ACTUATOR_ELEVATOR,
  HACS_ACTUATOR_RUDDER,
  HACS_ACTUATOR_AUX_0,
  HACS_ACTUATOR_AUX_1,
  HACS_ACTUATOR_AUX_2,
  HACS_ACTUATOR_AUX_3,

  HACS_NUM_ACTUATOR,
} hacs_actuator_t;
extern const hacs_pwm_chan_t actuator_to_pwm_map[HACS_NUM_ACTUATOR];

/*** GPIO ***/

// The CS pin for SPI
#define NRF24_CS_PORT                   GPIOA
#define NRF24_CS_PIN                    GPIO_PIN_1
// CE pin for NRF24L01
#define NRF24_CE_PORT                   GPIOA
#define NRF24_CE_PIN                    GPIO_PIN_4
// IRQ pin for NRF24L01
#define NRF24_IRQ_PORT                  GPIOC
#define NRF24_IRQ_PIN                   GPIO_PIN_1

#define ADS1120_CS_PORT                 GPIOC
#define ADS1120_CS_PIN                  GPIO_PIN_0

#define ADS1120_DRDY_PORT               GPIOC
#define ADS1120_DRDY_PIN                GPIO_PIN_4

#define ST_TO_TINY_PORT                 GPIOD
#define ST_TO_TINY_PIN                  GPIO_PIN_2

/* Definition for USART2 clock resources */
#define USART2_RX_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()
#define USART2_TX_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()

/* Definition for USART2 Pins */
#define USART2_TX_PIN                   GPIO_PIN_2
#define USART2_TX_PORT                  GPIOA
#define USART2_TX_AF                    GPIO_AF7_USART2
#define USART2_RX_PIN                   GPIO_PIN_3
#define USART2_RX_PORT                  GPIOA
#define USART2_RX_AF                    GPIO_AF7_USART2

/* Definition for USART1 clock resources */
#define USART1_RX_GPIO_CLK_ENABLE()     __GPIOB_CLK_ENABLE()
#define USART1_TX_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()

/* Definition for USART1 Pins */
#define USART1_TX_PIN                   GPIO_PIN_15
#define USART1_TX_PORT                  GPIOA
#define USART1_TX_AF                    GPIO_AF7_USART1
#define USART1_RX_PIN                   GPIO_PIN_7
#define USART1_RX_PORT                  GPIOB
#define USART1_RX_AF                    GPIO_AF7_USART1

/* Definition for USART6 clock resources */
#define USART6_RX_GPIO_CLK_ENABLE()     __GPIOC_CLK_ENABLE()
#define USART6_TX_GPIO_CLK_ENABLE()     __GPIOC_CLK_ENABLE()

/* Definition for USART6 Pins */
#define USART6_TX_PIN                   GPIO_PIN_6
#define USART6_TX_PORT                  GPIOC
#define USART6_TX_AF                    GPIO_AF8_USART6
#define USART6_RX_PIN                   GPIO_PIN_7
#define USART6_RX_PORT                  GPIOC
#define USART6_RX_AF                    GPIO_AF8_USART6

/* Definition for SPI2 resources */
#define SPI2_MOSI_GPIO_CLK_ENABLE()     __GPIOB_CLK_ENABLE()
#define SPI2_MISO_GPIO_CLK_ENABLE()     __GPIOB_CLK_ENABLE()
#define SPI2_SCK_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()

/* Definition for SPI2 pins */
#define SPI2_MOSI_PORT                  GPIOB
#define SPI2_MOSI_PIN                   GPIO_PIN_15
#define SPI2_MOSI_AF                    GPIO_AF5_SPI2
#define SPI2_MISO_PORT                  GPIOB
#define SPI2_MISO_PIN                   GPIO_PIN_14
#define SPI2_MISO_AF                    GPIO_AF5_SPI2
#define SPI2_SCK_PORT                   GPIOB
#define SPI2_SCK_PIN                    GPIO_PIN_13
#define SPI2_SCK_AF                     GPIO_AF5_SPI2

/* Definition for SPI3 resources */
#define SPI3_MOSI_GPIO_CLK_ENABLE()     __GPIOC_CLK_ENABLE()
#define SPI3_MISO_GPIO_CLK_ENABLE()     __GPIOC_CLK_ENABLE()
#define SPI3_SCK_GPIO_CLK_ENABLE()      __GPIOC_CLK_ENABLE()

/* Definition for SPI3 pins */
#define SPI3_MOSI_PORT                  GPIOC
#define SPI3_MOSI_PIN                   GPIO_PIN_12
#define SPI3_MOSI_AF                    GPIO_AF6_SPI3
#define SPI3_MISO_PORT                  GPIOC
#define SPI3_MISO_PIN                   GPIO_PIN_11
#define SPI3_MISO_AF                    GPIO_AF6_SPI3
#define SPI3_SCK_PORT                   GPIOC
#define SPI3_SCK_PIN                    GPIO_PIN_10
#define SPI3_SCK_AF                     GPIO_AF6_SPI3

/* Definition for I2C1 resources */
#define I2C1_SDA_GPIO_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
#define I2C1_SCL_GPIO_CLK_ENABLE()        __GPIOB_CLK_ENABLE()

/* Definition for I2C1 pins */
#define I2C1_SDA_PORT                   GPIOB
#define I2C1_SDA_PIN                    GPIO_PIN_9
#define I2C1_SDA_AF                     GPIO_AF4_I2C1
#define I2C1_SCL_PORT                   GPIOB
#define I2C1_SCL_PIN                    GPIO_PIN_8
#define I2C1_SCL_AF                     GPIO_AF4_I2C1

/* Definition for TIM PWM output pins */
#define TIM1_CHAN1_PORT                 GPIOA
#define TIM1_CHAN1_PIN                  GPIO_PIN_8
#define TIM1_CHAN1_AF                   GPIO_AF1_TIM1
#define TIM1_CHAN2_PORT                 GPIOA
#define TIM1_CHAN2_PIN                  GPIO_PIN_9
#define TIM1_CHAN2_AF                   GPIO_AF1_TIM1
#define TIM1_CHAN3_PORT                 GPIOA
#define TIM1_CHAN3_PIN                  GPIO_PIN_10
#define TIM1_CHAN3_AF                   GPIO_AF1_TIM1
#define TIM1_CHAN4_PORT                 GPIOA
#define TIM1_CHAN4_PIN                  GPIO_PIN_11
#define TIM1_CHAN4_AF                   GPIO_AF1_TIM1

#define TIM3_CHAN1_PORT                 GPIOA
#define TIM3_CHAN1_PIN                  GPIO_PIN_6
#define TIM3_CHAN1_AF                   GPIO_AF2_TIM3
#define TIM3_CHAN2_PORT                 GPIOA
#define TIM3_CHAN2_PIN                  GPIO_PIN_7
#define TIM3_CHAN2_AF                   GPIO_AF2_TIM3
#define TIM3_CHAN3_PORT                 GPIOB
#define TIM3_CHAN3_PIN                  GPIO_PIN_0
#define TIM3_CHAN3_AF                   GPIO_AF2_TIM3
#define TIM3_CHAN4_PORT                 GPIOB
#define TIM3_CHAN4_PIN                  GPIO_PIN_1
#define TIM3_CHAN4_AF                   GPIO_AF2_TIM3

/* 
 * Definition for receiver channel pins.
 * Channel numbers correspond to those imprinted on the HACS board
 */
#define RC_CHAN1_PORT                   GPIOC
#define RC_CHAN1_PIN                    GPIO_PIN_9
#define RC_CHAN2_PORT                   GPIOC
#define RC_CHAN2_PIN                    GPIO_PIN_8
#define RC_CHAN3_PORT                   GPIOC
#define RC_CHAN3_PIN                    GPIO_PIN_5
#define RC_CHAN4_PORT                   GPIOA
#define RC_CHAN4_PIN                    GPIO_PIN_12
#define RC_CHAN5_PORT                   GPIOB
#define RC_CHAN5_PIN                    GPIO_PIN_6
#define RC_CHAN6_PORT                   GPIOB
#define RC_CHAN6_PIN                    GPIO_PIN_2

#endif
