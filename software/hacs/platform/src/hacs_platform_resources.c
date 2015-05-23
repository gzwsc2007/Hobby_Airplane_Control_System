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
