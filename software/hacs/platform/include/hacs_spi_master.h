#ifndef _HACS_SPI_MASTER_H_
#define _HACS_SPI_MASTER_H_

#include <stdlib.h>
#include "hacs_platform_resources.h"

#define HACS_SPI_CPOL_0   (0)
#define HACS_SPI_CPOL_1   (1)
#define HACS_SPI_CPHA_0   (0)
#define HACS_SPI_CPHA_1   (1)

// The type hacs_spi_t is defined in hacs_platform_resources.h

/* These functions should only be called from a task */
int spi_master_init(hacs_spi_t bus, uint32_t freq, uint8_t cpol, uint8_t cpha);
int spi_master_transfer(hacs_spi_t bus, uint8_t *wbuf, size_t wsize, uint8_t *rbuf, size_t rsize);
int spi_master_write(hacs_spi_t bus, uint8_t *wbuf, size_t wsize);
int spi_master_read(hacs_spi_t bus, uint8_t *rbuf, size_t rsize);
int spi_master_exchange(hacs_spi_t bus, uint8_t *wbuf, uint8_t *rbuf, size_t size);

void spi_master_assert_cs(hacs_spi_t bus);
void spi_master_deassert_cs(hacs_spi_t bus);

#endif
