#include <string.h>
#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_platform_resources.h"
#include "hacs_spi_master.h"
#include "hacs_uart.h"
#include "hacs_debug_uart.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "hmc5883.h"
#include "nrf24l01.h"
#include "gps_serial.h"
#include "mpu6050_serial.h"
#include "bmp085.h"
#include "hacs_sensor_sched.h"
#include "hacs_timer.h"
#include "rc_receiver.h"
#include "hacs_pstore.h"
#include "ms4525do.h"
#include "ads1120.h"

extern uint8_t g_sensor_log_enable;

static uint8_t radio_test[32] = {
  'b', 'e', 'e', 'f', '\0'
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

static int bmp_retval;
static uint8_t bmp_done;
static void bmp_cb(int ret) {
  bmp_retval = ret;
  bmp_done = 1;
}

static void timer_cb(void) {
  printf("tim!\r\n");
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
    while (!debug_uart_wait_rxne()) {
      hmc5883_update_xyz(&magX, &magY, &magZ);
      printf("%hd %hd %hd\r\n", magX, magY, magZ);
      vTaskDelay(MS_TO_TICKS(150));
    }

  } else if (!strcmp(buf, "r")) {
    retval = nrf24_send(radio_test, sizeof(radio_test), NRF24_ACK);

  } else if (!strcmp(buf, "rdump")) {
    nrf24_dump_registers();

  } else if (!strcmp(buf, "gps raw")) {
    uint8_t buf[128];
    gps_ht = 0;
    hacs_uart_start_listening(HACS_UART_GPS, (uint32_t)buf, 128, gps_ht_cb, gps_tc_cb);
    while (!debug_uart_wait_rxne()) {
      while (!gps_ht);
      gps_ht = 0;
      gps_tc = 0;
      for (int i = 0; i < gps_rx_len; i++) {
        debug_uart_putchar(buf[i]);
      }
      while (!gps_tc);
      for (int i = gps_rx_len; i < 128; i++) {
        debug_uart_putchar(buf[i]);
      }
    }
    hacs_uart_stop_listening(HACS_UART_GPS);

  } else if (!strcmp(buf, "gps+")) {
    gps_data_t temp;
    xQueueHandle q = gps_get_msg_queue();

    gps_start_parsing();
    while (!debug_uart_wait_rxne()) {
      xQueueReceive(q, &temp, portMAX_DELAY);
      printf("lat: %d long: %d speed: %d course: %d\r\n",
             temp.latitude, temp.longitude, temp.speed, temp.course);
    }
    gps_stop_parsing();

  } else if (!strcmp(buf, "mpu raw")) {
    uint8_t buf[128];
    mpu_ht = 0;
    hacs_uart_start_listening(HACS_UART_MPU6050, (uint32_t)buf, 128, mpu_ht_cb, mpu_tc_cb);
    while (!debug_uart_wait_rxne()) {
      while (!mpu_ht);
      mpu_ht = 0;
      mpu_tc = 0;
      for (int i = 0; i < mpu_rx_len; i++) {
        debug_uart_putchar(buf[i]);
      }
      while (!mpu_tc);
      for (int i = mpu_rx_len; i < 128; i++) {
        debug_uart_putchar(buf[i]);
      }
    }
    hacs_uart_stop_listening(HACS_UART_MPU6050);
  } else if (!strcmp(buf, "mpu+")) {
    mpu_data_t temp;
    xQueueHandle q = mpu6050_get_msg_queue();

    mpu6050_start_parsing(MPU_DRIVER_CONTINUOUS_MODE);
    while (!debug_uart_wait_rxne()) {
      xQueueReceive(q, &temp, portMAX_DELAY);
      printf("ro: %d pi: %d ya: %d\r\n",
             (int16_t)temp.roll, (int16_t)temp.pitch, (int16_t)temp.yaw);
    }
    mpu6050_stop_parsing();

  } else if (!strcmp(buf, "bmp")) {
    int16_t temperature;
    float altitude;

    bmp_done = 0;
    bmp085_request_sample(&altitude, &temperature, bmp_cb);
    while (!bmp_done) vTaskDelay(MS_TO_TICKS(5));
    if (bmp_retval == HACS_NO_ERROR) {
      printf("Temperature: %d (0.1 C)\t Altitude: %d (0.1 m)\r\n", temperature, (int)(altitude * 10.0));
    } else {
      printf("Error: %d\r\n", bmp_retval);
    }
  } else if (!strcmp(buf, "gyro stats")) { /*
    mpu_data_t temp;
    xQueueHandle q = mpu6050_get_msg_queue();
    float p_bias = 0.0f;
    float q_bias = 0.0f;
    float r_bias = 0.0f;
    float p_err = 0.0f;
    float q_err= 0.0f;
    float r_err = 0.0f;

    mpu6050_start_parsing(MPU_DRIVER_CONTINUOUS_MODE);
    for (int i = 0; i < NUM_SAMPLES; i++) {
      xQueueReceive(q, &temp, portMAX_DELAY);
      sensor_buf[0][i] = mpu.p;
      sensor_buf[1][i] = mpu.q;
      sensor_buf[2][i] = mpu.r;
    }
    mpu6050_stop_parsing();
  */} else if (!strcmp(buf, "sensor log")) {
    g_sensor_log_enable = !g_sensor_log_enable;
  } else if (!memcmp(buf, "tim ", sizeof("tim ")-1)) {
    buf += sizeof("tim ")-1;
    uint32_t us = strtoul(buf, NULL, 10);
    timer_set_update_cb(HACS_BASIC_TIMER, timer_cb);
    timer_set_period(HACS_BASIC_TIMER, us);
    timer_reset_n_go(HACS_BASIC_TIMER);

    while(!debug_uart_wait_rxne()) vTaskDelay(MS_TO_TICKS(50));

    timer_stop(HACS_BASIC_TIMER);

  } else if (!memcmp(buf, "pwm ", sizeof("pwm ")-1)) {
    buf += sizeof("pwm ")-1;
    float percent = (float)strtoul(buf, NULL, 10) / 100.0;
    timer_set_period(HACS_PWM_TIMER_0, 2000);
    timer_set_period(HACS_PWM_TIMER_1, 2000);

    timer_set_pwm_duty(HACS_PWM_CHAN_1, percent);
    timer_set_pwm_duty(HACS_PWM_CHAN_2, percent);
    timer_set_pwm_duty(HACS_PWM_CHAN_3, percent);
    timer_set_pwm_duty(HACS_PWM_CHAN_4, percent);
    timer_set_pwm_duty(HACS_PWM_CHAN_5, percent);
    timer_set_pwm_duty(HACS_PWM_CHAN_6, percent);
    timer_set_pwm_duty(HACS_PWM_CHAN_7, percent);
    timer_set_pwm_duty(HACS_PWM_CHAN_8, percent);

    timer_start_pwm(HACS_PWM_CHAN_1);
    timer_start_pwm(HACS_PWM_CHAN_2);
    timer_start_pwm(HACS_PWM_CHAN_3);
    timer_start_pwm(HACS_PWM_CHAN_4);
    timer_start_pwm(HACS_PWM_CHAN_5);
    timer_start_pwm(HACS_PWM_CHAN_6);
    timer_start_pwm(HACS_PWM_CHAN_7);
    timer_start_pwm(HACS_PWM_CHAN_8);

    while(!debug_uart_wait_rxne()) vTaskDelay(MS_TO_TICKS(50));

    timer_stop(HACS_PWM_TIMER_0);
    timer_stop(HACS_PWM_TIMER_1);

  } else if (!strcmp(buf, "rc")) {
    printf("throttle: %d\r\n", rc_recvr_read_chan_raw(RC_CHAN_THROTTLE));
    printf("aileron: %d\r\n", rc_recvr_read_chan_raw(RC_CHAN_AILERON));
    printf("elevator: %d\r\n", rc_recvr_read_chan_raw(RC_CHAN_ELEVATOR));
    printf("rudder: %d\r\n", rc_recvr_read_chan_raw(RC_CHAN_RUDDER));
    printf("aux0: %d\r\n", rc_recvr_read_chan_raw(RC_CHAN_AUX_0));
    printf("aux1: %d\r\n", rc_recvr_read_chan_raw(RC_CHAN_AUX_1));

  } else if (!strcmp(buf, "pstore test")) {
    uint8_t set_buf[] = {1,2,3,4,5,6,7,8};
    uint8_t get_buf[sizeof(set_buf)];
    retval = hacs_pstore_set(HACS_PSTORE_MAG_HARD_CAL, set_buf, sizeof(set_buf));
    HACS_REQUIRES(retval >= 0, done);
    retval = hacs_pstore_get(HACS_PSTORE_MAG_HARD_CAL, get_buf, sizeof(get_buf));
    printf("get: %d %d %d %d %d %d %d %d\r\n", get_buf[0], get_buf[1], get_buf[2],
           get_buf[3], get_buf[4], get_buf[5], get_buf[6], get_buf[7]);
  } else if (!strcmp(buf, "ms4525")) {
    float p;
    retval = ms4525do_get_dp(&p);
    HACS_REQUIRES(retval >= 0, done);
    printf("%f\r\n", p);

  } else if (!memcmp(buf, "adc ", sizeof("adc ")-1)) {
    buf += sizeof("adc ")-1;
    uint32_t chan = strtoul(buf, NULL, 10);
    uint16_t res = 0;
    HACS_REQUIRES(chan <= ADS1120_SINGLE_ENDED_CHAN_3, done);
    retval = ads1120_read_single_ended(chan, &res);
    HACS_REQUIRES(retval >= 0, done);
    printf("%4x\r\n", res);
  }

done:
  return retval;
}
