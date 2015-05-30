#include "hacs_platform.h"
#include "hacs_app.h"
#include "FreeRTOS.h"
#include "task.h"

#include "hacs_console.h"
#include "nrf24l01.h"
#include "hacs_telemetry.h"

void hacs_app_init(void)
{
	xTaskCreate(hacs_console_task, "console", 512, NULL, 6, NULL);
  xTaskCreate(nrf24_driver_task, "nrf24", 128, NULL, 2, NULL);
  xTaskCreate(hacs_telemetry_tx_task, "telem_tx", 512, NULL, 5, NULL);
  xTaskCreate(hacs_telemetry_rx_task, "telem_rx", 512, NULL, 5, NULL);
}
