#include <stdio.h>
#include <math.h>
#include "hacs_platform.h"
#include "hacs_sensor_sched.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "mpu6050_serial.h"
#include "gps_serial.h"
#include "bmp085.h"
#include "hmc5883.h"
#include "ms4525do.h"
#include "ads1120.h"
#include "hacs_system_config.h"
#include "hacs_telemetry.h"
#include "hacs_sysid.h"
#include "actuator.h"
#include "rc_receiver.h"

#include "arm_math.h"
#include "MadgwickAHRS.h"

#define MPU6050_TIMEOUT_MS    (10)
#define BMP085_TIMEOUT_MS     (30)

#define AIR_DENSITY_KG_M3           (1.15f) // kg/m^3

#define DECLINATIOIN_COMPENSATION   (-2.7f)

#define ALPHA_ACCEL     (0.7f)
#define ALPHA_GYRO      (0.05f)
#define ALPHA_AIRSPEED  (0.8f) // cutoff freq ~1 Hz with Ts == 40ms

#define ADC_BATTERY_VOLT_CHAN       (ADS1120_SINGLE_ENDED_CHAN_3)
// R_lower == 9960 ohm
// R_upper == 32820 ohm
#define BATTERY_DIVIDER_FACTOR      (4.295f) // (R_lower + R_upper) / R_lower

static int bmp085_retval;
static xSemaphoreHandle bmp085_done_sema4;
uint8_t g_sensor_log_enable = 0;

static void bmp085_done_cb(int retval);
static float get_airspeed(void);

void hacs_sensor_sched_task(void *param) {
  xQueueHandle mpu_msg_queue = mpu6050_get_msg_queue();
  xQueueHandle gps_msg_queue = gps_get_msg_queue();
  mpu_data_t mpu_data;
  gps_data_t gps_data;
  float airspeed = 0.0;
  float altitude;
  float batt_volt;
  float batt_current = 0;
  int16_t temperature;
  portBASE_TYPE gps_data_available;
  portBASE_TYPE bmp085_data_available;
  portTickType xLastWakeTime;
  hacs_mode_t mode;
  int16_t magx, magy, magz;
  float32_t calmx, calmy, calmz;
  float roll, pitch, yaw;
  float ax = 0;
  float ay = 0;
  float az = 0;
  float gx = 0;
  float gy = 0;
  float gz = 0;
  int retval;

  if (hmc5883_init() != 0) {
    printf("Error in hmc5883_init!\r\n");
  }

  bmp085_done_sema4 = xSemaphoreCreateBinary();
  xLastWakeTime = xTaskGetTickCount();

  gps_start_parsing(); // Always enable GPS parsing

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, MS_TO_TICKS(40));
    mode = hacs_get_system_mode();

    if (mode == HACS_MODE_MAG_CAL) {
      vTaskDelay(MS_TO_TICKS(60)); // only want this to run at 10Hz
      hmc5883_update_xyz(&magx, &magy, &magz);
      hacs_telem_send_magcal(magx, magy, magz);
      continue;
    }

    // Request a single reading from the MPU
    retval = mpu6050_start_parsing(MPU_DRIVER_SINGLESHOT_MODE);
    if (retval != 0) {
      printf("mpu %d\r\n", retval);
    }

    // Request a single reading from the barometer
    retval = bmp085_request_sample(&altitude, &temperature, bmp085_done_cb);
    if (retval != 0) {
      printf("bmp %d\r\n", retval);
    }

    // obtain filtered airspeed reading
    airspeed = get_airspeed() * (1.0-ALPHA_AIRSPEED) + airspeed * ALPHA_AIRSPEED;

    // obtain calibrated magnetic heading from compass
    hmc5883_xyz_calibrated(&calmx, &calmy, &calmz);

    // Poll GPS for data. Don't need to block because GPS is much slower.
    gps_data_available = xQueueReceive(gps_msg_queue, &gps_data, 0);

    xQueueReceive(mpu_msg_queue, &mpu_data, MS_TO_TICKS(MPU6050_TIMEOUT_MS));

    // Use a simple first order IIR filter to low-pass filter the accel and gyro readings
    gx = mpu_data.p * (1.0-ALPHA_GYRO) + gx * ALPHA_GYRO;
    gy = mpu_data.q * (1.0-ALPHA_GYRO) + gy * ALPHA_GYRO;
    gz = mpu_data.r * (1.0-ALPHA_GYRO) + gz * ALPHA_GYRO;
    ax = mpu_data.ax * (1.0-ALPHA_ACCEL) + ax * ALPHA_ACCEL;
    ay = mpu_data.ay * (1.0-ALPHA_ACCEL) + ay * ALPHA_ACCEL;
    az = mpu_data.az * (1.0-ALPHA_ACCEL) + az * ALPHA_ACCEL;
    // Perform sensor fusion on accel, gyro, magnetometer readings.
    MadgwickAHRSupdate(gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,
                       ax, ay, az,
                       calmx, calmy, calmz);

    // Obtain battery readings (driven by GPS which has a lower sampling rate)
    if (gps_data_available) {
      ads1120_read_single_ended(ADC_BATTERY_VOLT_CHAN, ADS1120_REF_EXTERNAL, &batt_volt);
      batt_volt *= BATTERY_DIVIDER_FACTOR;
    }

    // Wait for BMP085 to finish
    bmp085_data_available = xSemaphoreTake(bmp085_done_sema4, MS_TO_TICKS(BMP085_TIMEOUT_MS));
    if (bmp085_data_available == pdTRUE) {
      if (bmp085_retval != HACS_NO_ERROR) {
        bmp085_data_available = pdFALSE;
        printf("bad bmp result %d\r\n", bmp085_retval);
      }
    } else {
      printf("bmp timeout\r\n");
    }

    // TODO: fuse MPU temperature reading and BMP temperature reading

    /*** At this point, all the sensor data are ready ***/

    // Translate quaternions to euler angles
    roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
    yaw   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    roll = roll * 180.0f / PI;
    pitch = pitch * 180.0f / PI;
    yaw = yaw * 180.0f / PI;
    yaw = yaw + DECLINATIOIN_COMPENSATION;

    if (g_sensor_log_enable) {
      printf("%f\t%f\t%f\r\n%f\t%f\t%f\r\n%f\t%f\t%f\r\n\r\n",gx,gy,gz,ax,ay,az,calmx,calmy,calmz);
    }

    // TODO: pack sensor data into a generic form defined by the Controller?

    // TODO: Notify Controller

    // Notify telemetry TX
    hacs_telem_send_pfd(roll, pitch, yaw, altitude, airspeed, batt_current);

    if (gps_data_available) {
      // NavD packets are driven by GPS
      hacs_telem_send_navd(gps_data.latitude, gps_data.longitude, gps_data.speed,
                           gps_data.course, temperature, batt_volt);
    }

    // Conduct system identification experiemnt
    if (mode == HACS_MODE_SYSTEM_IDENTIFICATION) {
      hacs_sysid_mode_t sysid_mode = hacs_get_sysid_mode();
      int32_t aile_output, elev_output, rudd_output;
      portTickType tick_count = xTaskGetTickCount();

      // Generate output value if we are not in manual mode
      if (sysid_mode != HACS_SYSID_MODE_MANUAL) {
        int32_t actuator_output;

        // Generate an output value
        retval = hacs_sysid_generate_output(tick_count, SYSTEM_IDENT_AMPLITUDE, &actuator_output);
        if (retval < 0) {
          // Time to terminate experiment. Notify ground station that the current
          // system mode has changed.
          hacs_telem_send_syscmd(HACS_GND_CMD_GET_MODE, hacs_get_system_mode());
        }
      }

      // Obtain actual output with trim removed (centered around trim values)
      aile_output = actuator_get_output_us(HACS_ACTUATOR_AILERON) - rc_recvr_get_trim_val(RC_CHAN_AILERON);
      elev_output = actuator_get_output_us(HACS_ACTUATOR_ELEVATOR) - rc_recvr_get_trim_val(RC_CHAN_ELEVATOR);
      rudd_output = actuator_get_output_us(HACS_ACTUATOR_RUDDER) - rc_recvr_get_trim_val(RC_CHAN_RUDDER);

      // Report data to ground station. Use raw data for accelerations and angule velocities.
      // Filtering can be done in post-processing.
      hacs_telem_send_sysid(tick_count, aile_output, elev_output, rudd_output,
                            mpu_data.ax, mpu_data.ay, mpu_data.az, 
                            roll, pitch, yaw,
                            mpu_data.p, mpu_data.q, mpu_data.r);
    }
  }
}

static void bmp085_done_cb(int retval) {
  bmp085_retval = retval;
  xSemaphoreGive(bmp085_done_sema4);
}

static float get_airspeed(void) {
  float dp = 0;
  float speed;

  ms4525do_get_dp_calibrated(&dp);
 
  if (dp > 0.0) {
    arm_sqrt_f32(2 * dp / AIR_DENSITY_KG_M3, &speed);
  } else {
    speed = 0.0;
  }

  return speed;
}
