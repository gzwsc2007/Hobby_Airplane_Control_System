#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "stm32f4xx_hal.h"

// Revise this table to allocate SPI peripherals
SPI_TypeDef* const hacs_spi_instances[HACS_NUM_SPI_PERIPH] = {
	[HACS_SPI_NRF24] = SPI3,
	[HACS_SPI_ADS1120] = SPI2,
};

GPIO_TypeDef* const hacs_spi_cs_port[HACS_NUM_SPI_PERIPH] = {
	[HACS_SPI_NRF24] = NRF24_CS_PORT,
	[HACS_SPI_ADS1120] = ADS1120_CS_PORT,
};

const uint16_t hacs_spi_cs_pin[HACS_NUM_SPI_PERIPH] = {
	[HACS_SPI_NRF24] = NRF24_CS_PIN,
	[HACS_SPI_ADS1120] = ADS1120_CS_PIN,
};

// Revise this table to allocate I2C peripherals
I2C_TypeDef* const hacs_i2c_instances[HACS_NUM_I2C_PERIPH] = {
	[HACS_I2C] = I2C1,
};

// Revise this table to allocate UART peripherals
USART_TypeDef* const hacs_uart_instances[HACS_NUM_UART_PERIPH] = {
	[HACS_UART_MPU6050] = USART6,
	[HACS_UART_GPS] = USART1,
};

DMA_Stream_TypeDef* const hacs_uart_rx_dma_stream[HACS_NUM_UART_PERIPH] = {
	[HACS_UART_MPU6050] = DMA2_Stream1,
	[HACS_UART_GPS] = DMA2_Stream2,
};

const uint32_t hacs_uart_rx_dma_chan[HACS_NUM_UART_PERIPH] = {
	[HACS_UART_MPU6050] = DMA_CHANNEL_5,
	[HACS_UART_GPS] = DMA_CHANNEL_4,
};

const IRQn_Type hacs_uart_rx_dma_irq[HACS_NUM_UART_PERIPH] = {
	[HACS_UART_MPU6050] = DMA2_Stream1_IRQn,
	[HACS_UART_GPS] = DMA2_Stream2_IRQn,
};

// Revise this table to allocate TIM peripherals
TIM_TypeDef* const hacs_tim_instances[HACS_NUM_TIMER_PERIPH] = {
	[HACS_BASIC_TIMER] = TIM5, // This has to be a 32-bit counter
	[HACS_PWM_TIMER_0] = TIM1,
	[HACS_PWM_TIMER_1] = TIM3,
};

// Revise this table to change mapping between RC input channels and GPIO pins.
// Note: Channel numbers correspond to those imprinted on the HACS board.
const gpio_port_pin_t rc_chan_to_gpio_map[HACS_NUM_RC_CHAN] = {
  [RC_CHAN_THROTTLE] = {RC_CHAN1_PORT, RC_CHAN1_PIN},
  [RC_CHAN_AILERON] = {RC_CHAN2_PORT, RC_CHAN2_PIN},
  [RC_CHAN_ELEVATOR] = {RC_CHAN3_PORT, RC_CHAN3_PIN},
  [RC_CHAN_RUDDER] = {RC_CHAN4_PORT, RC_CHAN4_PIN},
  [RC_CHAN_AUX_0] = {RC_CHAN5_PORT, RC_CHAN5_PIN},
  [RC_CHAN_AUX_1] = {RC_CHAN6_PORT, RC_CHAN6_PIN},
};

// Revise this table to change the mapping between actuator output channels and pwm channels
const hacs_pwm_chan_t actuator_to_pwm_map[HACS_NUM_ACTUATOR] = {
	[HACS_ACTUATOR_THROTTLE] = HACS_PWM_CHAN_8,
	[HACS_ACTUATOR_AILERON] = HACS_PWM_CHAN_7,
	[HACS_ACTUATOR_ELEVATOR] = HACS_PWM_CHAN_6,
	[HACS_ACTUATOR_RUDDER] = HACS_PWM_CHAN_5,
	[HACS_ACTUATOR_AUX_0] = HACS_PWM_CHAN_4,
	[HACS_ACTUATOR_AUX_1] = HACS_PWM_CHAN_3,
	[HACS_ACTUATOR_AUX_2] = HACS_PWM_CHAN_2,
	[HACS_ACTUATOR_AUX_3] = HACS_PWM_CHAN_1,
};
