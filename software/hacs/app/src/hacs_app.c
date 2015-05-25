#include "hacs_platform.h"
#include "hacs_app.h"
#include "FreeRTOS.h"
#include "task.h"

#include "hacs_console.h"
#include "nrf24l01.h"

void hacs_app_init(void)
{
	xTaskCreate(hacs_console_task, "console", 1024, NULL, 6, NULL );
  xTaskCreate(nrf24_driver_task, "nrf24", 256, NULL, 2, NULL);
}
