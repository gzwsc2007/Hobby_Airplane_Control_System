#include <stdio.h>
#include "hacs_platform.h"
#include "hacs_sensor_manager.h"
#include "queue.h"
#include "semphr.h"

#include "mpu6050_serial.h"
#include "gps_serial.h"
#include "bmp085.h"
#include "hacs_system_config.h"
#include "hacs_telemetry.h"

#define MPU6050_TIMEOUT_MS    (100)
#define BMP085_TIMEOUT_MS     (50)

static int bmp085_retval;
static xSemaphoreHandle bmp085_done_sema4;

static void bmp085_done_cb(int retval);

void hacs_sensor_manager_task(void *param) {
  xQueueHandle mpu_msg_queue = mpu6050_get_msg_queue();
  xQueueHandle gps_msg_queue = gps_get_msg_queue();
  mpu_data_t mpu_data;
  gps_data_t gps_data;
  float altitude;
  int16_t temperature;
  portBASE_TYPE gps_data_available;
  portBASE_TYPE bmp085_data_available;
  portTickType xLastWakeTime;
  int retval;

  bmp085_done_sema4 = xSemaphoreCreateBinary();
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, MS_TO_TICKS(40));

    // Request a single reading from the MPU
    mpu6050_start_parsing(MPU_DRIVER_SINGLESHOT_MODE);

    // Request a single reading from the barometer
    retval = bmp085_request_sample(&altitude, &temperature, bmp085_done_cb);
    if (retval != HACS_NO_ERROR) {
      continue;
    }

    // TODO: request airspeed reading

    // TODO: obtain calibrated magnetic heading from compass

    // TODO: fuse the compass heading and the MPU yaw

    // Poll GPS for data. Don't need to block because GPS is much slower.
    gps_data_available = xQueueReceive(gps_msg_queue, &gps_data, 0);

    xQueueReceive(mpu_msg_queue, &mpu_data, portMAX_DELAY);

    // Wait for BMP085 to finish
    bmp085_data_available = xSemaphoreTake(bmp085_done_sema4, portMAX_DELAY);
    if (bmp085_data_available == pdTRUE) {
      if (bmp085_retval != HACS_NO_ERROR) {
        bmp085_data_available = pdFALSE;
      }
    }

    // TODO: fuse MPU temperature reading and BMP temperature reading

    /*** At this point, all the sensor data are ready ***/

    // TODO: pack sensor data into a generic form defined by the Controller?

    // TODO: Notify Controller

    // Notify telemetry TX
    hacs_telem_send_pfd(mpu_data.roll, mpu_data.pitch, mpu_data.yaw, altitude, 0, 0);

    if (gps_data_available) {
      // NavD packets are driven by GPS
      hacs_telem_send_navd(gps_data.latitude, gps_data.longitude, gps_data.speed,
                           gps_data.course, temperature, 0);
    }
    
    // TODO: decide what to send (e.g. send SysID packets?) depending on system state
    if (hacs_get_system_mode() == HACS_MODE_SYSTEM_IDENTIFICATION) {

    }
  }
}

int hacs_sensor_manager_start() {
  int retval;

  retval = gps_start_parsing();
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);

done:
  return retval;
}

int hacs_sensor_manager_stop() {
  int retval;

  retval = gps_stop_parsing();
  HACS_REQUIRES(retval == HACS_NO_ERROR, done);
  
done:
  return retval;
}

static void bmp085_done_cb(int retval) {
  bmp085_retval = retval;
  xSemaphoreGive(bmp085_done_sema4);
}
