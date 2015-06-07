#ifndef _MPU6050_SERIAL_H_
#define _MPU6050_SERIAL_H_

#include "hacs_platform.h"
#include "queue.h"

#define MPU_DATA_QUEUE_LENGTH   (1)

#define MPU_DRIVER_SINGLESHOT_MODE  (1)
#define MPU_DRIVER_CONTINUOUS_MODE  (0)

typedef struct {
  // Acceleration in g (Note that the coordinate system used here is
  // different than the standard airplane body-axis system)
  float ax;
  float ay;
  float az;
  // Attitude in deg
  float roll;
  float pitch;
  float yaw;
  // Angular velocities in deg/s
  float rollspeed;
  float pitchspeed;
  float yawspeed;
  // Temperature in C
  float temperature;
} mpu_data_t;

int mpu6050_early_init(void);

xQueueHandle mpu6050_get_msg_queue(void);

int mpu6050_start_parsing(uint8_t singleshot);

int mpu6050_stop_parsing();

#endif