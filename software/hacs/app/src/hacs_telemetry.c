#include <assert.h>
#include "hacs_platform.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "hacs_telemetry.h"
#include "nrf24l01.h"
#include "mavlink.h"
#include "hacs_system_config.h"
#include "hmc5883.h"
#include "bmp085.h"
#include "ms4525do.h"
#include "rc_receiver.h"
#include "hacs_sysid.h"

#define SYSTEM_ID     (250)
#define COMPONENT_ID  (125)

static xQueueHandle tx_queue;
static uint8_t pfd_struct_busy;
static uint8_t navd_struct_busy;
static uint8_t magcal_struct_busy;
static uint8_t sysid_struct_busy;
static uint8_t syscmd_struct_busy;
static mavlink_pfd_t pfd_s;
static mavlink_navd_t navd_s;
static mavlink_magcal_t magcal_s;
static mavlink_systemid_t sysid_s;
static mavlink_syscmd_t syscmd_s;
// TODO: To save RAM, for RX, instead of having a struct for each message type,
// I can have a single buffer that's large enough to hold any type
// of message struct, and then just reuse the buffer and cast to
// the appropriate struct each time. This is possible because we will
// only ever be processing one RX packet at a time
static mavlink_syscmd_t syscmd_rx_s;
static mavlink_magcalresult_t magcalresult_s;

static mavlink_status_t mv_parser_status;
static mavlink_message_t rx_msg;
static mavlink_message_t tx_msg;
static uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];

static int radio_send_wrapper(uint8_t *data, uint32_t len);
static int handle_syscmd(mavlink_syscmd_t *syscmd);
static void handle_telem_test(uint32_t count);

int hacs_telemetry_early_init() {
  pfd_struct_busy = 0;
  navd_struct_busy = 0;
  magcal_struct_busy = 0;
  tx_queue = xQueueCreate(HACS_TELEM_TX_QUEUE_LEN, sizeof(int));
  return HACS_NO_ERROR;
}

void hacs_telemetry_rx_task(void *param) {
  nrf24_msg_t rx;
  xQueueHandle nrf24_queue = nrf24_get_msg_queue();
  uint8_t idx;

  while (1) {
    xQueueReceive(nrf24_queue, &rx, portMAX_DELAY);

    for (idx = 0; idx < rx.len; idx++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, rx.buf[idx], &rx_msg, &mv_parser_status)) {
        // Handle the message
        switch (rx_msg.msgid) {
        case MAVLINK_MSG_ID_SysCmd:
          mavlink_msg_syscmd_decode(&rx_msg, &syscmd_rx_s);
          handle_syscmd(&syscmd_rx_s);
          break;
        case MAVLINK_MSG_ID_MagCalResult:
          mavlink_msg_magcalresult_decode(&rx_msg, &magcalresult_s);
          hmc5883_set_cal(magcalresult_s.hard_iron, magcalresult_s.soft_iron);
          break;
        default: break;
        }
      }
    }
  }
}

// Periodically reports flight data to ground station
void hacs_telemetry_tx_task(void *param) {
  tx_type_t tx;
  uint32_t len;

  while (1) {
    xQueueReceive(tx_queue, &tx, portMAX_DELAY);

    switch (tx) {
    case TELEM_TX_PFD:
      assert(pfd_struct_busy);
      mavlink_msg_pfd_encode(SYSTEM_ID, COMPONENT_ID, &tx_msg, &pfd_s);
      pfd_struct_busy = 0;
      break;
    case TELEM_TX_NAVD:
      assert(navd_struct_busy);
      mavlink_msg_navd_encode(SYSTEM_ID, COMPONENT_ID, &tx_msg, &navd_s);
      navd_struct_busy = 0;
      break;
    case TELEM_TX_MAGCAL:
      assert(magcal_struct_busy);
      mavlink_msg_magcal_encode(SYSTEM_ID, COMPONENT_ID, &tx_msg, &magcal_s);
      magcal_struct_busy = 0;
      break;
    case TELEM_TX_SYSID:
      assert(sysid_struct_busy);
      mavlink_msg_systemid_encode(SYSTEM_ID, COMPONENT_ID, &tx_msg, &sysid_s);
      sysid_struct_busy = 0;
      break;
    case TELEM_TX_SYSCMD:
      assert(syscmd_struct_busy);
      mavlink_msg_syscmd_encode(SYSTEM_ID, COMPONENT_ID, &tx_msg, &syscmd_s);
      syscmd_struct_busy = 0;
      break;
    default:
      break;
    }

    len = mavlink_msg_to_send_buffer(tx_buf, &tx_msg);
    radio_send_wrapper(tx_buf, len);
  }
}

static int radio_send_wrapper(uint8_t *data, uint32_t len) {
  uint8_t *ptr = data;
  int retval;

  while (len >= NRF24_MAX_MESSAGE_LEN)
  {
    retval = nrf24_send(ptr, NRF24_MAX_MESSAGE_LEN, NRF24_ACK);
    HACS_REQUIRES(retval == HACS_NO_ERROR, done);
    len -= NRF24_MAX_MESSAGE_LEN;
    ptr += NRF24_MAX_MESSAGE_LEN;
  }

  // handle the rest of the data
  if (len > 0) {
    retval = nrf24_send(ptr, len, NRF24_ACK);
  }

done:
  return retval;
}

int hacs_telem_send_pfd(float roll, float pitch, float yaw,
                        float alt, float airspeed, float batt_I) {
  const tx_type_t type = TELEM_TX_PFD;

  if (pfd_struct_busy) {
    return -HACS_ERR_ALREADY_IN_USE;
  }
  pfd_struct_busy = 1;

  pfd_s.roll = (int16_t)(roll * 100.0); // 0.01 deg
  pfd_s.pitch = (int16_t)(pitch * 100.0); // 0.01 deg
  pfd_s.yaw = (int16_t)(yaw * 100.0); // 0.01 deg
  pfd_s.altitude = (int16_t)(alt * 10.0); // 0.1 m
  pfd_s.airspeed = (int16_t)(airspeed * 10.0); // 0.1 m/s
  pfd_s.battI = (int16_t)(batt_I * 100.0); // 0.01 A

  // Notify TX task
  xQueueSend(tx_queue, &type, portMAX_DELAY);

  return HACS_NO_ERROR;
}

int hacs_telem_send_navd(int32_t latitude, int32_t longitude, uint16_t speed,
                         uint16_t course, int16_t temperature, float batt_V) {
  const tx_type_t type = TELEM_TX_NAVD;

  if (navd_struct_busy) {
    return -HACS_ERR_ALREADY_IN_USE;
  }
  navd_struct_busy = 1;

  navd_s.latitude = latitude;
  navd_s.longitude = longitude;
  navd_s.groundspeed = speed;
  navd_s.course = course;
  navd_s.battV = (int16_t)(batt_V * 100.0); // 0.01 V
  navd_s.temp = temperature;
  
  // Notify TX task
  xQueueSend(tx_queue, &type, portMAX_DELAY);

  return HACS_NO_ERROR;
}

int hacs_telem_send_magcal(int16_t magx, int16_t magy, int16_t magz) {
  const tx_type_t type = TELEM_TX_MAGCAL;

  if (magcal_struct_busy) {
    return -HACS_ERR_ALREADY_IN_USE;
  }
  magcal_struct_busy = 1;

  magcal_s.mx = magx;
  magcal_s.my = magy;
  magcal_s.mz = magz;

  xQueueSend(tx_queue, &type, portMAX_DELAY);

  return HACS_NO_ERROR;
}

int hacs_telem_send_syscmd(uint8_t cmd, uint32_t payload) {
  const tx_type_t type = TELEM_TX_SYSCMD;

  if (syscmd_struct_busy) {
    return -HACS_ERR_ALREADY_IN_USE;
  }
  syscmd_struct_busy = 1;

  syscmd_s.cmd = cmd;
  syscmd_s.payload = payload;

  xQueueSend(tx_queue, &type, portMAX_DELAY);

  return HACS_NO_ERROR; 
}

int hacs_telem_send_sysid(uint32_t timestamp,
                          int32_t u_a, int32_t u_e, int32_t u_r,
                          float ax, float ay, float az,
                          float roll, float pitch, float yaw,
                          float p, float q, float r) {
  const tx_type_t type = TELEM_TX_SYSID;

  if (sysid_struct_busy) {
    return -HACS_ERR_ALREADY_IN_USE;
  }
  sysid_struct_busy = 1;

  sysid_s.timestamp = timestamp;
  sysid_s.u_a = u_a;
  sysid_s.u_e = u_e;
  sysid_s.u_r = u_r;
  sysid_s.ax = (int16_t)(ax * 100.0f); // in 0.01g
  sysid_s.ay = (int16_t)(ay * 100.0f); // in 0.01g
  sysid_s.az = (int16_t)(az * 100.0f); // in 0.01g
  sysid_s.roll = (int16_t)(roll * 100.0f); // in 0.01 deg
  sysid_s.pitch = (int16_t)(pitch * 100.0f); // in 0.01 deg
  sysid_s.yaw = (int16_t)(yaw * 100.0f); // in 0.01 deg
  sysid_s.p = (int16_t)(p * 10.0f); // in 0.1 deg/s
  sysid_s.q = (int16_t)(q * 10.0f); // in 0.1 deg/s
  sysid_s.r = (int16_t)(r * 10.0f); // in 0.1 deg/s

  xQueueSend(tx_queue, &type, portMAX_DELAY);

  return HACS_NO_ERROR;
}

static int handle_syscmd(mavlink_syscmd_t *syscmd) {
  int retval = 0;
  uint32_t payload = syscmd->payload;

  switch (syscmd->cmd) {
  case HACS_GND_CMD_SET_MODE:
    if (payload == HACS_MODE_SYSTEM_IDENTIFICATION &&
        hacs_get_sysid_mode() != HACS_SYSID_MODE_MANUAL) {
      // Start the system id process
      hacs_sysid_start(xTaskGetTickCount(), SYSTEM_IDENT_DURATION_US);
      vTaskDelay(MS_TO_TICKS(20));
    }
    hacs_set_system_mode((hacs_mode_t)payload);
    break;
  case HACS_GND_CMD_GET_MODE:
    break;
  case HACS_GND_CMD_CALIBRATE_BAROMETER:
    bmp085_ground_calibration();
    break;
  case HACS_GND_CMD_CALIBRATE_AIRSPEED:
    ms4525do_zero_cal();
    break;
  case HACS_GND_CMD_CALIBRATE_TRIM_VALUES:
    rc_recvr_set_trim_vals();
    break;
  case HACS_GND_CMD_SET_SYSID_MODE:
    hacs_set_sysid_mode((hacs_sysid_mode_t)payload);
    break;
  case HACS_GND_CMD_SET_SYSID_FREQ:
    hacs_set_sysid_freq((hacs_sysid_freq_t)payload);
    break;
  case HACS_GND_CMD_TELEM_TEST:
    handle_telem_test(payload);
    break;
  default: break;
  }

  return retval;
}

static void handle_telem_test(uint32_t count) {
  static uint32_t my_count = 0;

  if (count == 0xFFFFFFFF) {
    // restart the test
    my_count = 0;
    return;
  }

  // Report packet loss
  hacs_telem_send_syscmd(HACS_GND_CMD_TELEM_TEST, my_count);
  my_count++;
}
