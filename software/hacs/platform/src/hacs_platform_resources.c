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
