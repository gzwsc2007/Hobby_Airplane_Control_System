#include <assert.h>
#include "hacs_platform.h"
#include "queue.h"
#include "hacs_telemetry.h"
#include "nrf24l01.h"
#include "mavlink.h"
#include "hacs_system_config.h"

#define SYSTEM_ID     (250)
#define COMPONENT_ID  (125)

static xQueueHandle tx_queue;
static uint8_t pfd_struct_busy;
static uint8_t navd_struct_busy;
static uint8_t magcal_struct_busy;
static mavlink_pfd_t pfd_s;
static mavlink_navd_t navd_s;
static mavlink_magcal_t magcal_s;
static mavlink_syscmd_t syscmd_s;

static mavlink_message_t rx_msg;
static mavlink_message_t tx_msg;
static uint8_t tx_buf[MAVLINK_MAX_PACKET_LEN];

static int radio_send_wrapper(uint8_t *data, uint32_t len);
static int handle_syscmd(mavlink_syscmd_t *syscmd);

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
  mavlink_status_t status;

  while (1) {
    xQueueReceive(nrf24_queue, &rx, portMAX_DELAY);

    for (idx = 0; idx < rx.len; idx++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, rx.buf[idx], &rx_msg, &status)) {
        // Handle the message
        switch (rx_msg.msgid) {
        case MAVLINK_MSG_ID_SysCmd:
          mavlink_msg_syscmd_decode(&rx_msg, &syscmd_s);
          handle_syscmd(&syscmd_s);
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
    return HACS_ERR_ALREADY_IN_USE;
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
    return HACS_ERR_ALREADY_IN_USE;
  }
  navd_struct_busy = 1;

  navd_s.latitude = latitude;
  navd_s.longitude = longitude;
  navd_s.groundspeed = speed;
  navd_s.course = course;
  navd_s.battV = (int16_t)(batt_V * 100.0); // 0.01 V

  // Notify TX task
  xQueueSend(tx_queue, &type, portMAX_DELAY);

  return HACS_NO_ERROR;
}

int hacs_telem_send_magcal(int16_t magx, int16_t magy, int16_t magz) {
  const tx_type_t type = TELEM_TX_MAGCAL;

  if (magcal_struct_busy) {
    return HACS_ERR_ALREADY_IN_USE;
  }
  magcal_struct_busy = 1;

  magcal_s.mx = magx;
  magcal_s.my = magy;
  magcal_s.mz = magz;

  xQueueSend(tx_queue, &type, portMAX_DELAY);

  return HACS_NO_ERROR;
}

static int handle_syscmd(mavlink_syscmd_t *syscmd) {
  int retval = 0;
  uint32_t payload = syscmd->payload;

  switch (syscmd->cmd) {
  case HACS_GND_CMD_SET_MODE:
    hacs_set_system_mode((hacs_mode_t)payload);
    break;
  case HACS_GND_CMD_GET_MODE:
    break;
  default: break;
  }

  return retval;
}
