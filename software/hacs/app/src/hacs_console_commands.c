#include <string.h>
#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_spi_master.h"
#include "hacs_uart.h"
#include "hacs_debug_uart.h"

#include "hmc5883.h"
#include "nrf24l01.h"

static uint8_t radio_test[32] = {
	'b','e','e','f','\0'
};

static uint8_t gps_rxne;
static uint32_t gps_rx_len;
static void gps_cb(uint32_t len) {
  gps_rx_len = len;
  gps_rxne = 1;
}

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
  } else if (!strcmp(buf, "gps")) {
    uint8_t buf[128];
    gps_rxne = 0;
    hacs_uart_start_listening(HACS_UART_GPS, (uint32_t)buf, 128, gps_cb);
    while(!debug_uart_inpstat()) {
      while(!gps_rxne);
      gps_rxne = 0;
      buf[127] = 0;
      printf("%s\r\n", buf);
    }
    hacs_uart_stop_listening(HACS_UART_GPS);
  }

  return retval;
}
