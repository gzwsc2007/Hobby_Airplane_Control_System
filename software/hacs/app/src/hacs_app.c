#include "hacs_platform.h"
#include "hacs_app.h"
#include "hacs_console.h"
#include "FreeRTOS.h"
#include "task.h"

void hacs_app_init(void)
{
	xTaskCreate(hacs_console_task, "console", 1024, NULL, 6, NULL );
}
