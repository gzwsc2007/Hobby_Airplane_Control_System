#include <string.h>
#include <stdio.h>
#include "hacs_platform_resources.h"
#include "hacs_spi_master.h"

#include "hmc5883.h"

int hacs_console_cmd_dispatch(char *buf)
{
  int retval = 0;

  if (!strcmp(buf, "mag")) {
  	uint16_t magX = 0;
  	uint16_t magY = 0;
  	uint16_t magZ = 0;
  	hmc5883_update_xyz(&magX, &magY, &magZ);
  	printf("%hd %hd %hd\n", magX, magY, magZ);
  }

  return retval;
}
