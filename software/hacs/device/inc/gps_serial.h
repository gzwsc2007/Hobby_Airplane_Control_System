#ifndef _GPS_SERIAL_H_
#define _GPS_SERIAL_H_

#include "hacs_platform.h"
#include "queue.h"

#define GPS_DATA_QUEUE_LENGTH   (2)

/*
 * Data in this strucct is compatible with the MAVlink protocol
 */
typedef struct {
  // Navigation succeeded or not.
  uint8_t valid;
  // Geographic position. Encoded as an integer with unit
  // 10^-7 degree
  int32_t latitude;   // positive - N , negative - S
  // Encoded as an integer with unit 10^-7 degree
  int32_t longitude;  // positive - E , negative - W
  // speed in 0.01 m/s
  uint16_t speed;
  // course in degree (0 deg corresponds to geographic North).
  // Encoded with unit 0.01 deg
  uint16_t course;
} gps_data_t;

int gps_early_init(void);

xQueueHandle gps_get_msg_queue(void);

int gps_start_parsing();

int gps_stop_parsing();

#endif
