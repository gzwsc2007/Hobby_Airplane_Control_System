#include <assert.h>
#include "hacs_platform.h"
#include "hacs_uart.h"
#include "mpu6050_serial.h"

#define MPU_RAW_BUF_LEN     80
#define MPU_PARSE_BUF_LEN   40

static xQueueHandle mpu_msg_queue;
static uint8_t raw_buf[MPU_RAW_BUF_LEN];
static uint32_t last_read_len;

static uint8_t parse_buf[MPU_PARSE_BUF_LEN];
static uint32_t parse_buf_ptr;

typedef enum {
  PARSER_STATE_IDLE = 0,
  PARSER_STATE_COPY_TO_LOCAL,
} parser_state_t;
static parser_state_t state;

static void mpu_ht_cb(uint32_t len_read);
static void mpu_tc_cb(uint32_t len_read);
static void mpu_parser_fsm(uint8_t *pdata, uint32_t len);
static int mpu_parse_buf(uint8_t *s, mpu_data_t *m);
static int mpu_validate_checksum(uint8_t *pdata);

int mpu6050_early_init(void) {
  mpu_msg_queue = xQueueCreate(MPU_DATA_QUEUE_LENGTH, sizeof(mpu_data_t));
  return 0;
}

xQueueHandle mpu6050_get_msg_queue(void) {
  return mpu_msg_queue;
}

int mpu6050_start_parsing() {
  parse_buf_ptr = 0;
  return hacs_uart_start_listening(HACS_UART_MPU6050, (uint32_t)raw_buf, sizeof(raw_buf),
                                   mpu_ht_cb, mpu_tc_cb);
}

int mpu6050_stop_parsing() {
  return hacs_uart_stop_listening(HACS_UART_MPU6050);
}

static void mpu_ht_cb(uint32_t len_read) {
  assert(len_read <= MPU_RAW_BUF_LEN);

  mpu_parser_fsm(raw_buf, len_read);
  last_read_len = len_read;
}

static void mpu_tc_cb(uint32_t len_read) {
  assert(last_read_len < MPU_RAW_BUF_LEN);

  mpu_parser_fsm(raw_buf+last_read_len, MPU_RAW_BUF_LEN-last_read_len);
}

static void mpu_parser_fsm(uint8_t *pdata, uint32_t len) {
  static uint8_t seen_header = 0;
  static uint8_t bytes_to_copy;
  uint8_t c;

  while(len > 0) {
    c = *pdata;

    // Feed the new byte to the parser state machine
    if (state == PARSER_STATE_IDLE) {
      // We only leave this state iff a valid accel packet header
      // has been found
      if (!seen_header) {
        if (c == 0x55) {
          seen_header = 1;
          parse_buf[parse_buf_ptr++] = c;
        }
      } else {
        if (c == 0x51) {
          parse_buf[parse_buf_ptr++] = c;
          state = PARSER_STATE_COPY_TO_LOCAL;
          bytes_to_copy = 31;
        } else {
          // This is not an accel packet. Reset local buffer.
          parse_buf_ptr = 0;
        }
        seen_header = 0;
      }

    } else if (state == PARSER_STATE_COPY_TO_LOCAL) {
      if (bytes_to_copy > 0) {
        parse_buf[parse_buf_ptr++] = c;
        bytes_to_copy--;
      } else {
        mpu_data_t temp_data;
        // We've copied 3 packets to local buffer. Validate and parse them.
        assert(parse_buf_ptr == 33);
        if (mpu_parse_buf(parse_buf, &temp_data) == HACS_NO_ERROR) {
          portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
          xQueueSendFromISR(mpu_msg_queue, &temp_data, &xHigherPriorityTaskWoken);
          if( xHigherPriorityTaskWoken ) {
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
          }
        }
        state = PARSER_STATE_IDLE;
        parse_buf_ptr = 0;
      }
    }

    pdata++;
    len--;
  }
}

static int mpu_parse_buf(uint8_t *buf, mpu_data_t *m) {
  uint8_t ptr = 0;
  
  if (mpu_validate_checksum(buf)) {
    return -1;
  }

  // decode angular vel and angle packet
  if (buf[ptr++] == 0x55)
  {
    // accel packet
    if (buf[ptr++] == 0x51) {
      m->ax = ((int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*16.0;
      ptr += 2;
      m->ay = ((int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*16.0;
      ptr += 2;
      m->az = ((int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*16.0;
      ptr += 5; // ignore temperature
    } else {
      return -1;
    }

    if (buf[ptr++] != 0x55)
      return -1;

    // angular vel packet
    if (buf[ptr++] == 0x52) {
      m->rollspeed = ((int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*2000.0;
      ptr += 2;
      m->pitchspeed = ((int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*2000.0;
      ptr += 2;
      m->yawspeed = ((int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*2000.0;
      ptr += 5; // ignore temperature
    } else {
      return -1;
    }
    
    if (buf[ptr++] != 0x55)
      return -1;
    
    // angle packet
    if (buf[ptr++] == 0x53) {
      m->roll = ((int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*180;
      ptr += 2;
      m->pitch = ((int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*180;
      ptr += 2;
      m->yaw = ((int16_t)(buf[ptr+1]<<8| buf[ptr]))/32768.0*180;
      ptr += 2;
      m->temperature = ((int16_t)(buf[ptr+1]<<8| buf[ptr]))/340.0+36.25;
      ptr += 3;
    } else {
      return -1;
    }

  } else { 
    return -1;
  }
  
  return 0;
}

static int mpu_validate_checksum(uint8_t *pdata) {
  uint8_t cnt;
  uint8_t checksum = 0;

  // Check first packet
  for (cnt = 10; cnt > 0; cnt--) {
    checksum += *pdata++;
  }

  if (checksum != *pdata++) {
    return -1;
  }
  checksum = 0;

  // Second packet
  for (cnt = 10; cnt > 0; cnt--) {
    checksum += *pdata++;
  }

  if (checksum != *pdata++) {
    return -1;
  }
  checksum = 0;

  // Third packet
  for (cnt = 10; cnt > 0; cnt--) {
    checksum += *pdata++;
  }

  if (checksum != *pdata++) {
    return -1;
  }

  return 0;
}