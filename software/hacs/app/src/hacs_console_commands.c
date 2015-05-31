#include <string.h>
#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_spi_master.h"
#include "hacs_uart.h"
#include "hacs_debug_uart.h"
#include "queue.h"

#include "hmc5883.h"
#include "nrf24l01.h"
#include "gps_serial.h"
#include "mpu6050_serial.h"

static uint8_t radio_test[32] = {
	'b','e','e','f','\0'
};

static uint8_t gps_ht;
static uint8_t gps_tc;
static uint32_t gps_rx_len;

static void gps_ht_cb(uint32_t len) {
  gps_ht = 1;
  gps_rx_len = len;
}

static void gps_tc_cb(uint32_t len) {
  gps_tc = 1;
}

static uint8_t mpu_ht;
static uint8_t mpu_tc;
static uint32_t mpu_rx_len;

static void mpu_ht_cb(uint32_t len) {
  mpu_ht = 1;
  mpu_rx_len = len;
}

static void mpu_tc_cb(uint32_t len) {
  mpu_tc = 1;
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

  } else if (!strcmp(buf, "gps raw")) {
    uint8_t buf[128];
    gps_ht = 0;
    hacs_uart_start_listening(HACS_UART_GPS, (uint32_t)buf, 128, gps_ht_cb, gps_tc_cb);
    while(!debug_uart_inpstat()) {
      while(!gps_ht);
      gps_ht = 0;
      gps_tc = 0;
      for (int i = 0; i < gps_rx_len; i++) {
        debug_uart_putchar(buf[i]);
      }
      while(!gps_tc);
      for (int i = gps_rx_len; i < 128; i++) {
        debug_uart_putchar(buf[i]);
      }
    }
    hacs_uart_stop_listening(HACS_UART_GPS);

  } else if (!strcmp(buf, "gps+")) {
    gps_data_t temp;
    xQueueHandle q = gps_get_msg_queue();

    gps_start_parsing();
    while(!debug_uart_inpstat()) {
      xQueueReceive(q, &temp, portMAX_DELAY);
      printf("lat: %d long: %d speed: %d course: %d\r\n",
             temp.latitude,temp.longitude,temp.speed,temp.course);
    }
    gps_stop_parsing();

  } else if (!strcmp(buf, "mpu raw")) {
    uint8_t buf[128];
    mpu_ht = 0;
    hacs_uart_start_listening(HACS_UART_MPU6050, (uint32_t)buf, 128, mpu_ht_cb, mpu_tc_cb);
    while(!debug_uart_inpstat()) {
      while(!mpu_ht);
      mpu_ht = 0;
      mpu_tc = 0;
      for (int i = 0; i < mpu_rx_len; i++) {
        debug_uart_putchar(buf[i]);
      }
      while(!mpu_tc);
      for (int i = mpu_rx_len; i < 128; i++) {
        debug_uart_putchar(buf[i]);
      }
    }
    hacs_uart_stop_listening(HACS_UART_MPU6050);
  } else if (!strcmp(buf, "mpu+")) {
    mpu_data_t temp;
    xQueueHandle q = mpu6050_get_msg_queue();

    mpu6050_start_parsing();
    while(!debug_uart_inpstat()) {
      xQueueReceive(q, &temp, portMAX_DELAY);
      printf("ro: %d pi: %d ya: %d\r\n",
             (int16_t)temp.roll,(int16_t)temp.pitch,(int16_t)temp.yaw);
    }
    mpu6050_stop_parsing();

  }

  return retval;
}
