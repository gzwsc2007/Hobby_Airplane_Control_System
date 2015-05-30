#include "hacs_platform.h"
#include "queue.h"
#include "hacs_telemetry.h"
#include "nrf24l01.h"
#include "mavlink.h"

static xQueueHandle tx_queue;

void hacs_telemetry_rx_task(void *param) {
  nrf24_msg_t rx;
  xQueueHandle nrf24_queue = nrf24_get_msg_queue();
  uint8_t idx;
  mavlink_message_t msg;
  mavlink_status_t status;

  while (1) {
    xQueueReceive(nrf24_queue, &rx, portMAX_DELAY);

    for (idx = 0; idx < rx.len; idx++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, rx.buf[idx], &msg, &status)) {
        // Handle the message
        switch (msg.msgid) {
          default: break;
        }
      }
    }
  }
}

// Periodically reports flight data to ground station
void hacs_telemetry_tx_task(void *param) {
  tx_type_t tx;

  tx_queue = xQueueCreate(HACS_TELEM_TX_QUEUE_LEN, sizeof(int));

  while (1) {
    xQueueReceive(tx_queue, &tx, portMAX_DELAY);

    switch (tx) {
      default: break;
    }
  }
}

int radio_send_wrapper(uint8_t *data, uint32_t len) 
{
  uint8_t *ptr = data;
  int retval;

  while (len >= NRF24_MAX_MESSAGE_LEN)
  {
      retval = nrf24_send(ptr, NRF24_MAX_MESSAGE_LEN, NRF24_ACK);
      if (retval != HACS_NO_ERROR) goto done;
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
