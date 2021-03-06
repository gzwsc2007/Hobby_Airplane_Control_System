#include "hacs_platform.h"
#include "hacs_app.h"
#include "FreeRTOS.h"
#include "task.h"

#include "hacs_console.h"
#include "hacs_telemetry.h"
#include "hacs_sensor_sched.h"
#include "hacs_pilot_cmd.h"

void hacs_app_init(void)
{
	xTaskCreate(hacs_console_task, "console", 256, NULL, 1, NULL);
	xTaskCreate(hacs_telemetry_tx_task, "telem_tx", 256, NULL, 4, NULL);
	xTaskCreate(hacs_telemetry_rx_task, "telem_rx", 256, NULL, 2, NULL);
	xTaskCreate(hacs_sensor_sched_task, "sensor_scheduler", 256, NULL, 3, NULL);
  xTaskCreate(hacs_pilot_cmd_task, "pilot_cmd", 128, NULL, 6, NULL);

  hacs_telemetry_early_init();
}
