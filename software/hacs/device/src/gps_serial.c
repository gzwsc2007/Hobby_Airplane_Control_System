#include <assert.h>
#include <string.h>
#include "hacs_platform.h"
#include "hacs_uart.h"
#include "FreeRTOS.h"
#include "gps_serial.h"

#define GPS_RAW_BUF_LEN     128
#define GPS_PARSE_BUF_LEN   128

static xQueueHandle gps_msg_queue;
static uint8_t raw_buf[GPS_RAW_BUF_LEN];
static volatile uint32_t last_read_len;

static uint8_t parse_buf[GPS_PARSE_BUF_LEN];
static volatile uint32_t parse_buf_ptr;

static void gps_ht_cb(uint32_t len_read);
static void gps_tc_cb(uint32_t len_read);
static void gps_parser_fsm(uint8_t *pdata, uint32_t len);
static int gps_parse_string(char *s, gps_data_t *g);
static int parse_GPRMC_block(char* b, int8_t i, gps_data_t* g);
/* Helper functions borrowed from:
    https://github.com/offchooffcho/STM32-1/blob/master/GPSTracker/car/nmea.c
 */
static int NMEA_convertLatLong(char *b, int32_t *p_result);
static int32_t NMEA_atoi(const char *s);

int gps_early_init(void) {
  gps_msg_queue = xQueueCreate(GPS_DATA_QUEUE_LENGTH, sizeof(gps_data_t));
  return 0;
}

xQueueHandle gps_get_msg_queue(void) {
  return gps_msg_queue;
}

int gps_start_parsing() {
  parse_buf_ptr = 0;
  return hacs_uart_start_listening(HACS_UART_GPS, (uint32_t)raw_buf, sizeof(raw_buf),
                                   gps_ht_cb, gps_tc_cb);
}

int gps_stop_parsing() {
  return hacs_uart_stop_listening(HACS_UART_GPS);
}

static void gps_ht_cb(uint32_t len_read) {
  assert(len_read <= GPS_RAW_BUF_LEN);

  gps_parser_fsm(raw_buf, len_read);
  last_read_len = len_read;
}

static void gps_tc_cb(uint32_t len_read) {
  assert(last_read_len < GPS_RAW_BUF_LEN);

  gps_parser_fsm(raw_buf + last_read_len, GPS_RAW_BUF_LEN - last_read_len);
}

static void gps_parser_fsm(uint8_t *pdata, uint32_t len) {
  uint8_t c;

  // Copy data to local buffer.
  while (len > 0) {
    c = *pdata;

    if (c == '\r' || c == '\n') {
      gps_data_t temp_data;

      // Got a complete message. Parse the local buffer
      parse_buf[parse_buf_ptr] = '\0';
      if (gps_parse_string((char*)parse_buf, &temp_data) == HACS_NO_ERROR) {
        portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(gps_msg_queue, &temp_data, &xHigherPriorityTaskWoken);
        if ( xHigherPriorityTaskWoken ) {
          portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        }
      }

      // Reset local buffer pointer. Ready for next message
      parse_buf_ptr = 0;

    } else {
      assert(parse_buf_ptr < GPS_PARSE_BUF_LEN);
      parse_buf[parse_buf_ptr++] = c;
    }

    pdata++;
    len--;
  }
}

/*
 * Take an NMEA string, parse it, and store the interpreted information in the
 * GPS_Data structure.
 *
 * Supports GPRMC for now.
 */
static int gps_parse_string(char *s, gps_data_t *g) {
  // a block for interpretation
  char *block;
  int8_t index = 1;  // block index
  int retval = -HACS_GPS_FORMAT_NOT_SUPPORTED;

  // determine the format of the string (can be used to extend to other format)
  if (strncmp(s, "$GPRMC", 6)) {
    return -HACS_GPS_FORMAT_NOT_SUPPORTED;
  }

  s += 7;
  block = s;
  while (*s != '\0')
  {
    if (*s == ',')
    {
      *s = '\0';  // set the "end" of a block
      retval = parse_GPRMC_block(block, index, g);
      HACS_REQUIRES(retval >= 0, done);
      s++;
      index++;
      block = s;
      continue;
    }
    s++;
  }

done:
  return retval;;
}

/* Helper function to parse GPRMC string */
static int parse_GPRMC_block(char* b, int8_t i, gps_data_t* g) {
  int32_t temp;
  int retval = HACS_NO_ERROR;

  if (*b == '\0') return retval;

  switch (i)
  {
  // Navigation Status: A = valid, V = invalid
  case 2:
    if (*b == 'A') g->valid = 1;
    else g->valid = 0;
    break;
  // Latitude
  case 3:
    retval = NMEA_convertLatLong(b, &g->latitude);
    break;
  // N or S Hemisphere
  case 4:
    if (*b == 'S') g->latitude = -(g->latitude);  // Use negative sign to denote S
    break;
  // Longitude
  case 5:
    retval = NMEA_convertLatLong(b, &g->longitude);
    break;
  // E or W Hemisphere
  case 6:
    if (*b == 'W') g->longitude = -(g->longitude);
    break;
  // Ground speed
  case 7:
    temp = NMEA_atoi(b);
    g->speed = (uint16_t)(temp * 643 / 12500); // convert 1 knot to 0.01 m/s
    break;
  // Course
  case 8:
    g->course = (uint16_t)NMEA_atoi(b);
    break;
  }

  return retval;
}

/*
 * Helper function to convert raw GPRMC lat/long string to an integer
 * with unit 10^-7 degree.
 */
static int NMEA_convertLatLong(char *b, int32_t* p_result) {
  char *tmp = strchr(b, '.'); // locate the '.' char
  double min;
  int minDec;
  int deg;
  int result;

  if (tmp == NULL) {
    return -HACS_GPS_FORMAT_NOT_SUPPORTED;
  }

  // convert the decimal portion of min
  minDec = NMEA_atoi(tmp + 1);
  min = ((double)minDec) / 100000.0;
  *tmp = '\0';

  // convert minute into the form of mm.mmmm
  tmp = tmp - 2; // go back to the first 'm'
  min += (double)NMEA_atoi(tmp);

  // convert degree into a single integer
  *tmp = '\0'; // safe to modify b in this case
  deg = NMEA_atoi(b);

  // scaling
  result = deg * 10000000; // scale 1 deg to 10^-7 deg
  min = (min / 60.0) * 10000000.0; // convert minute to 10^-7 deg

  *p_result = result + (int32_t)min;

  return HACS_NO_ERROR;
}

/*
 * Convert strings of the format "ddmm.mmmm" into integer.
 * Borrowed from
 * https://github.com/offchooffcho/STM32-1/blob/master/GPSTracker/car/nmea.c
 */
static int32_t NMEA_atoi(const char *s)
{
  int32_t result = 0;

  while ((*s >= '0' && *s <= '9') || *s == '.')
  {
    if (*s == '.')
    {
      s++;
      continue;
    }
    result *= 10;
    result += *s - '0';
    s++;
  }

  return result;
}
