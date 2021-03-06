#ifndef _HACS_TELEMETRY_H_
#define _HACS_TELEMETRY_H_

typedef enum {
  TELEM_TX_PFD = 1,
  TELEM_TX_NAVD,
  TELEM_TX_SYSID,
  TELEM_TX_MAGCAL,
  TELEM_TX_SYSCMD,
} tx_type_t;

#define HACS_TELEM_TX_QUEUE_LEN  5

int hacs_telemetry_early_init();

void hacs_telemetry_rx_task(void *param);
void hacs_telemetry_tx_task(void *param);

int hacs_telem_send_pfd(float roll, float pitch, float yaw,
                        float alt, float airspeed, float batt_I);
int hacs_telem_send_navd(int32_t latitude, int32_t longitude, uint16_t speed,
                         uint16_t course, int16_t temperature, float batt_V);
int hacs_telem_send_magcal(int16_t magx, int16_t magy, int16_t magz);
int hacs_telem_send_sysid(uint32_t timestamp,
                          int32_t u_a, int32_t u_e, int32_t u_r,
                          float ax, float ay, float az,
                          float roll, float pitch, float yaw,
                          float p, float q, float r);
int hacs_telem_send_syscmd(uint8_t cmd, uint32_t payload);

#endif
