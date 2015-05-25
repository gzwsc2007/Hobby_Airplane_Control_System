#include <string.h>
#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_spi_master.h"
#include "hacs_debug_uart.h"

#include "hmc5883.h"
#include "nrf24l01.h"

/*
static uint8_t radio_test[32] = {
	31,
	1,2,3,4,5,6,7,8,9,
	1,2,3,4,5,6,7,8,9,
	1,2,3,4,5,6,7,8,9,
  0xA,0xA,0xA,0xA
};*/
  static uint8_t radio_test[32] = {
	'b','e','e','f','\0'
};

int hacs_console_cmd_dispatch(char *buf)
{
  int retval = 0;

  if (!strcmp(buf, "mag")) {
  	int16_t magX = 0;
  	int16_t magY = 0;
  	int16_t magZ = 0;
  	hmc5883_update_xyz(&magX, &magY, &magZ);
  	printf("%hd %hd %hd\r\n", magX, magY, magZ);
  } else if (!strcmp(buf, "mag+")) {
  	int16_t magX = 0;
  	int16_t magY = 0;
  	int16_t magZ = 0;
  	while(!debug_uart_inpstat()) {
  		hmc5883_update_xyz(&magX, &magY, &magZ);
  		printf("%hd %hd %hd\r\n", magX, magY, magZ);
  		vTaskDelay(MS_TO_TICKS(150));
  	}
  } else if (!strcmp(buf, "r")) {
  	retval = nrf24_send(radio_test,sizeof(radio_test),NRF24_ACK);
  } else if (!strcmp(buf, "rdump")) {
  	nrf24_dump_registers();
  }

  return retval;
}
